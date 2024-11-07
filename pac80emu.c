#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <unistd.h>

#include <SDL2/SDL.h>

#include "8080/i8080.h"
#include "emu76489/emu76489.h"

#define VA15  (1 << 0)
#define VINTE (1 << 1)
#define UINTE (1 << 2)
#define KINT  (1 << 3)
#define KSTB  (1 << 4)
#define KINTE (1 << 4)
#define KIBF  (1 << 5)
#define VINT  (1 << 6)
#define UINT  (1 << 7)
#define TXRDY (1 << 0)
#define RXRDY (1 << 1)
#define UP    (1 << 0)
#define DOWN  (1 << 1)
#define LEFT  (1 << 2)
#define RIGHT (1 << 3)
#define AB    (1 << 4)
#define STRTC (1 << 5)
#define SEL   (1 << 6)

#define BUTTON_U (1 << 0)
#define BUTTON_D (1 << 1)
#define BUTTON_L (1 << 2)
#define BUTTON_R (1 << 3)
#define BUTTON_B (1 << 5)
#define BUTTON_C (1 << 4)
#define BUTTON_A (1 << 6)
#define BUTTON_S (1 << 7)
#define BUTTON_Z (1 << 8)
#define BUTTON_Y (1 << 9)
#define BUTTON_X (1 << 10)
#define BUTTON_M (1 << 11)

typedef struct FIFO FIFO;
struct FIFO{
	uint8_t buf[256];
	uint16_t head;
	uint16_t tail;
	uint8_t s;
};

typedef struct Machine Machine;
struct Machine{
	uint8_t *ram;
	uint8_t *rom;
	uint8_t *map[4];
	uint8_t uart_rx;
	uint8_t uart_tx;
	uint8_t uart_status;
	FIFO uart_fifo;
	uint16_t cf_scount;
	uint16_t cf_bcount;
	uint32_t cf_lba;
	uint8_t cf_status;
	uint8_t *cf_data;
	off_t cf_size;
	uint8_t ppi_a;
	uint8_t ppi_b;
	uint8_t ppi_c;
	FIFO kb_fifo;
	SNG *sng;
	uint16_t js_buttons;
	uint8_t js_state;
	uint8_t js_timer;
};

static const uint8_t js_guid[] = {
	0x05, 0x00, 0x00, 0x00, 0x4c, 0x05, 0x00, 0x00,
	0xc4, 0x05, 0x00, 0x00, 0x00, 0x81, 0x00, 0x00
};

static const uint16_t js_map[] = {
	[0] = BUTTON_B,
	[1] = BUTTON_C,
	[2] = BUTTON_Y,
	[3] = BUTTON_A,
	[4] = BUTTON_X,
	[5] = BUTTON_Z,
	[8] = BUTTON_M,
	[9] = BUTTON_S,
};

static inline uint16_t
fifo_space(FIFO *f)
{
	return f->tail - f->head;
}

static inline uint16_t
fifo_count(FIFO *f)
{
	return (sizeof(f->buf) >> f->s) - fifo_space(f);
}

static inline void
fifo_push(FIFO *f, uint8_t data)
{
	if(fifo_space(f))
		f->buf[f->head++ & ((sizeof(f->buf) >> f->s) - 1)] = data;
}

static inline uint8_t
fifo_pop(FIFO *f)
{
	return f->buf[f->tail++ & ((sizeof(f->buf) >> f->s) - 1)];
}

static uint8_t
read_byte(void *userdata, uint16_t addr)
{
	Machine *m;

	m = userdata;
	return m->map[addr >> 14][addr & 0x3fff];
}

static void
write_byte(void *userdata, uint16_t addr, uint8_t val)
{
	Machine *m;

	m = userdata;
	if(m->map[addr >> 14] != m->rom)
		m->map[addr >> 14][addr & 0x3fff] = val;
}

static uint8_t
port_in(void *userdata, uint8_t port)
{
	Machine *m;
	uint8_t d;

	m = userdata;
//	printf("read port %02x\n", port);
	switch(port & 0x38){
	case 0x08:	/* BANK */
		if(m->map[port >> 6] == m->rom)
			return 0xff;
		else
			return ((m->map[port >> 6] - m->ram) >> 14) | 0xf0;
	case 0x28:	/* UART */
		switch(port & 1){
		case 0:	/* data */
			d = m->uart_rx;
			m->uart_status &= ~RXRDY;
			m->ppi_c &= ~UINT;
			if(fifo_count(&m->uart_fifo)){
				m->uart_rx = fifo_pop(&m->uart_fifo);
				m->uart_status |= RXRDY;
				if(m->ppi_c & UINTE)
					m->ppi_c |= UINT;
			}
			return d;
		case 1: /* status */
			return m->uart_status;
		}
	case 0x30:	/* CF */
		switch(port & 7){
		case 0:	/* data */
			d = *(m->cf_data + m->cf_lba * 512 + m->cf_bcount);
			m->cf_bcount++;
			if(m->cf_bcount == 512){
				m->cf_bcount = 0;
				m->cf_scount--;
				if(m->cf_scount == 0)
					m->cf_status = 0;
				else
					m->cf_lba++;
			}
			return d;
		case 1: /* error */
			return 0;
		case 2:	/* sector count */
			return m->cf_scount & 0xff;
		case 3: /* lba0 */
			return m->cf_lba & 0xff;
		case 4:	/* lba1 */
			return (m->cf_lba >> 8) & 0xff;
		case 5: /* lba2 */
			return (m->cf_lba >> 16) & 0xff;
		case 6:	/* lba3 */
			return ((m->cf_lba >> 24) & 0x0f) | 0xe0;
		case 7: /* status */
			if((m->cf_status & 0x08) && (m->cf_lba * 512 + m->cf_bcount >= m->cf_size))
				m->cf_status = 0x01;
			return m->cf_status;
		}
	case 0x18:	/* PPI */
		switch(port & 5){
		case 0: /* port a */
			m->ppi_c &= ~(KIBF | KINT);
			return m->ppi_a;
		case 1: /* port b */
			d = m->ppi_b;
			if((port & 2) && !(m->ppi_b & SEL)){
				m->js_state = (m->js_state + 1) & 3;
				m->js_timer = 0;
				m->ppi_b |= UP | DOWN | LEFT | RIGHT | AB | STRTC;
				if(m->js_state == 3){
					if(m->js_buttons & BUTTON_Z)
						m->ppi_b &= ~UP;
					if(m->js_buttons & BUTTON_Y)
						m->ppi_b &= ~DOWN;
					if(m->js_buttons & BUTTON_X)
						m->ppi_b &= ~LEFT;
					if(m->js_buttons & BUTTON_M)
						m->ppi_b &= ~RIGHT;
					if(m->js_buttons & BUTTON_B)
						m->ppi_b &= ~AB;
					if(m->js_buttons & BUTTON_C)
						m->ppi_b &= ~STRTC;
				}else{
					if(m->js_buttons & BUTTON_U)
						m->ppi_b &= ~UP;
					if(m->js_buttons & BUTTON_D)
						m->ppi_b &= ~DOWN;
					if(m->js_buttons & BUTTON_L)
						m->ppi_b &= ~LEFT;
					if(m->js_buttons & BUTTON_R)
						m->ppi_b &= ~RIGHT;
					if(m->js_buttons & BUTTON_B)
						m->ppi_b &= ~AB;
					if(m->js_buttons & BUTTON_C)
						m->ppi_b &= ~STRTC;
				}
				m->ppi_b |= SEL;
			}else if(!(port & 2) && (m->ppi_b & SEL)){
				m->ppi_b |= UP | DOWN | LEFT | RIGHT | AB | STRTC;
				if(m->js_state == 2){
					m->ppi_b &= ~(UP | DOWN | LEFT | RIGHT);
					if(m->js_buttons & BUTTON_A)
						m->ppi_b &= ~AB;
					if(m->js_buttons & BUTTON_S)
						m->ppi_b &= ~STRTC;
				}else if(m->js_state == 3){
					if(m->js_buttons & BUTTON_A)
						m->ppi_b &= ~AB;
					if(m->js_buttons & BUTTON_S)
						m->ppi_b &= ~STRTC;
				}else{
					m->ppi_b &= ~(LEFT | RIGHT);
					if(m->js_buttons & BUTTON_U)
						m->ppi_b &= ~UP;
					if(m->js_buttons & BUTTON_D)
						m->ppi_b &= ~DOWN;
					if(m->js_buttons & BUTTON_A)
						m->ppi_b &= ~AB;
					if(m->js_buttons & BUTTON_S)
						m->ppi_b &= ~STRTC;
				}
				m->ppi_b &= ~SEL;
			}
			return d;
		case 4: /* port c */
			d = m->ppi_c;
			m->ppi_c &= ~VINT;
			return d;
		case 5: /* illegal */
			break;
		}
	case 0x38:	/* PSG */
	case 0x00:	/* EXT0 */
	case 0x10:	/* EXT1 */
	case 0x20:	/* EXT2 */
		break;
	}
	return 0xff;
}

static void
port_out(void *userdata, uint8_t port, uint8_t val)
{
	Machine *m;
	uint8_t bit;

	m = userdata;
//	printf("write port %02x val %02x\n", port, val);
	switch(port & 0x38){
	case 0x08:	/* BANK */
		if((val & 0xf) == 0xf)
			m->map[port >> 6] = m->rom;
		else
			m->map[port >> 6] = m->ram + (((uint32_t)val & 0xf) << 14);
		break;
	case 0x28:	/* UART */
		switch(port & 1){
		case 0:	/* data */
			m->uart_status &= ~TXRDY;
			m->uart_tx = val;
			break;
		case 1: /* control */
			break;
		}
		break;
	case 0x30:	/* CF */
		switch(port & 7){
		case 0: /* data */
			*(m->cf_data + m->cf_lba * 512 + m->cf_bcount) = val;
			m->cf_bcount++;
			if(m->cf_bcount == 512){
				m->cf_bcount = 0;
				m->cf_scount--;
				if(m->cf_scount == 0)
					m->cf_status = 0;
				else
					m->cf_lba++;
			}
			break;
		case 1: /* feature */
			break;
		case 2:	/* sector count */
			m->cf_scount = val;
			break;
		case 3: /* lba0 */
			m->cf_lba = (m->cf_lba & 0xffffff00) | ((uint32_t)val << 0);
			break;
		case 4:	/* lba1 */
			m->cf_lba = (m->cf_lba & 0xffff00ff) | ((uint32_t)val << 8);
			break;
		case 5: /* lba2 */
			m->cf_lba = (m->cf_lba & 0xff00ffff) | ((uint32_t)val << 16);
			break;
		case 6:	/* lba3 */
			m->cf_lba = (m->cf_lba & 0x00ffffff) | ((uint32_t)(val & 0x0f) << 24);
			break;
		case 7: /* command */
			switch(val){
			case 0x20: /* read sectors */
			case 0x30: /* write sectors */
				if(m->cf_scount == 0)
					m->cf_scount = 256;
				m->cf_bcount = 0;
				m->cf_status = 0x08;
				break;
			case 0xef: /* set features */
				break;
			}
			break;
		}
		break;
	case 0x18:	/* PPI */
		switch(port & 5){
		case 0: /* port a */
		case 1: /* port b */
			break;
		case 4: /* port c */
			m->ppi_c = (m->ppi_c & 0xe8) | (val & 0x17);
			m->ppi_c &= ~UINT;
			m->ppi_c |= ((m->uart_status & RXRDY) << 6) & ((m->ppi_c & UINTE) << 5);
			break;
		case 5: /* control */
			if((val & 0x80) == 0){
				bit = 1 << ((val >> 1) & 7);
				if(val & 1)
					val = m->ppi_c | bit;
				else
					val = m->ppi_c & ~bit;
				m->ppi_c = (m->ppi_c & 0xe8) | (val & 0x17);
				m->ppi_c &= ~UINT;
				m->ppi_c |= ((m->uart_status & RXRDY) << 6) & ((m->ppi_c & UINTE) << 5);
			}
			break;
		}
	case 0x38:	/* PSG */
		SNG_writeIO(m->sng, val);
		break;
	case 0x00:	/* EXT0 */
	case 0x10:	/* EXT1 */
	case 0x20:	/* EXT2 */
		break;
	}
}

static const uint8_t xlat[SDL_NUM_SCANCODES] = {
	[SDL_SCANCODE_A]            = 0x1e,
	[SDL_SCANCODE_B]            = 0x30,
	[SDL_SCANCODE_C]            = 0x2e,
	[SDL_SCANCODE_D]            = 0x20,
	[SDL_SCANCODE_E]            = 0x12,
	[SDL_SCANCODE_F]            = 0x21,
	[SDL_SCANCODE_G]            = 0x22,
	[SDL_SCANCODE_H]            = 0x23,
	[SDL_SCANCODE_I]            = 0x17,
	[SDL_SCANCODE_J]            = 0x24,
	[SDL_SCANCODE_K]            = 0x25,
	[SDL_SCANCODE_L]            = 0x26,
	[SDL_SCANCODE_M]            = 0x32,
	[SDL_SCANCODE_N]            = 0x31,
	[SDL_SCANCODE_O]            = 0x18,
	[SDL_SCANCODE_P]            = 0x19,
	[SDL_SCANCODE_Q]            = 0x10,
	[SDL_SCANCODE_R]            = 0x13,
	[SDL_SCANCODE_S]            = 0x1f,
	[SDL_SCANCODE_T]            = 0x14,
	[SDL_SCANCODE_U]            = 0x16,
	[SDL_SCANCODE_V]            = 0x2f,
	[SDL_SCANCODE_W]            = 0x11,
	[SDL_SCANCODE_X]            = 0x2d,
	[SDL_SCANCODE_Y]            = 0x15,
	[SDL_SCANCODE_Z]            = 0x2c,
	[SDL_SCANCODE_1]            = 0x02,
	[SDL_SCANCODE_2]            = 0x03,
	[SDL_SCANCODE_3]            = 0x04,
	[SDL_SCANCODE_4]            = 0x05,
	[SDL_SCANCODE_5]            = 0x06,
	[SDL_SCANCODE_6]            = 0x07,
	[SDL_SCANCODE_7]            = 0x08,
	[SDL_SCANCODE_8]            = 0x09,
	[SDL_SCANCODE_9]            = 0x0a,
	[SDL_SCANCODE_0]            = 0x0b,
	[SDL_SCANCODE_RETURN]       = 0x1c,
	[SDL_SCANCODE_ESCAPE]       = 0x01,
	[SDL_SCANCODE_BACKSPACE]    = 0x0e,
	[SDL_SCANCODE_TAB]          = 0x0f,
	[SDL_SCANCODE_SPACE]        = 0x39,
	[SDL_SCANCODE_MINUS]        = 0x0c,
	[SDL_SCANCODE_EQUALS]       = 0x0d,
	[SDL_SCANCODE_LEFTBRACKET]  = 0x1a,
	[SDL_SCANCODE_RIGHTBRACKET] = 0x1b,
	[SDL_SCANCODE_BACKSLASH]    = 0x2b,
	[SDL_SCANCODE_NONUSHASH]    = 0x00,
	[SDL_SCANCODE_SEMICOLON]    = 0x27,
	[SDL_SCANCODE_APOSTROPHE]   = 0x28,
	[SDL_SCANCODE_GRAVE]        = 0x29,
	[SDL_SCANCODE_COMMA]        = 0x33,
	[SDL_SCANCODE_PERIOD]       = 0x34,
	[SDL_SCANCODE_SLASH]        = 0x35,
	[SDL_SCANCODE_CAPSLOCK]     = 0x3a,
	[SDL_SCANCODE_F1]           = 0x3b,
	[SDL_SCANCODE_F2]           = 0x3c,
	[SDL_SCANCODE_F3]           = 0x3d,
	[SDL_SCANCODE_F4]           = 0x3e,
	[SDL_SCANCODE_F5]           = 0x3d,
	[SDL_SCANCODE_F6]           = 0x40,
	[SDL_SCANCODE_F7]           = 0x41,
	[SDL_SCANCODE_F8]           = 0x42,
	[SDL_SCANCODE_F9]           = 0x43,
	[SDL_SCANCODE_F10]          = 0x44,
	[SDL_SCANCODE_F11]          = 0x57,
	[SDL_SCANCODE_F12]          = 0x58,
	[SDL_SCANCODE_PRINTSCREEN]  = 0x37,
	[SDL_SCANCODE_SCROLLLOCK]   = 0x46,
	[SDL_SCANCODE_PAUSE]        = 0x45,
	[SDL_SCANCODE_INSERT]       = 0x52,
	[SDL_SCANCODE_HOME]         = 0x47,
	[SDL_SCANCODE_PAGEUP]       = 0x49,
	[SDL_SCANCODE_DELETE]       = 0x53,
	[SDL_SCANCODE_END]          = 0x4f,
	[SDL_SCANCODE_PAGEDOWN]     = 0x51,
	[SDL_SCANCODE_RIGHT]        = 0x4d,
	[SDL_SCANCODE_LEFT]         = 0x4b,
	[SDL_SCANCODE_DOWN]         = 0x50,
	[SDL_SCANCODE_UP]           = 0x48,
	[SDL_SCANCODE_NUMLOCKCLEAR] = 0x45,
	[SDL_SCANCODE_KP_DIVIDE]    = 0x35,
	[SDL_SCANCODE_KP_MULTIPLY]  = 0x37,
	[SDL_SCANCODE_KP_MINUS]     = 0x4a,
	[SDL_SCANCODE_KP_PLUS]      = 0x4e,
	[SDL_SCANCODE_KP_ENTER]     = 0x1c,
	[SDL_SCANCODE_KP_1]         = 0x4f,
	[SDL_SCANCODE_KP_2]         = 0x50,
	[SDL_SCANCODE_KP_3]         = 0x51,
	[SDL_SCANCODE_KP_4]         = 0x4b,
	[SDL_SCANCODE_KP_5]         = 0x4c,
	[SDL_SCANCODE_KP_6]         = 0x4d,
	[SDL_SCANCODE_KP_7]         = 0x47,
	[SDL_SCANCODE_KP_8]         = 0x48,
	[SDL_SCANCODE_KP_9]         = 0x49,
	[SDL_SCANCODE_KP_0]         = 0x52,
	[SDL_SCANCODE_KP_PERIOD]    = 0x53,
    [SDL_SCANCODE_APPLICATION]  = 0x5d,
	[SDL_SCANCODE_SYSREQ]       = 0x54,
	[SDL_SCANCODE_LCTRL]        = 0x1d,
	[SDL_SCANCODE_LSHIFT]       = 0x2a,
	[SDL_SCANCODE_LALT]         = 0x38,
	[SDL_SCANCODE_LGUI]         = 0x5b,
	[SDL_SCANCODE_RCTRL]        = 0x1d,
	[SDL_SCANCODE_RSHIFT]       = 0x36,
	[SDL_SCANCODE_RALT]         = 0x38,
	[SDL_SCANCODE_RGUI]         = 0x5c,
};

static void
reset(Machine *m)
{
	m->map[0] = m->rom;
	m->map[1] = m->rom;
	m->map[2] = m->rom;
	m->map[3] = m->rom;

	m->uart_status = TXRDY;
	m->uart_fifo.head = 0;
	m->uart_fifo.tail = sizeof(m->uart_fifo.buf) >> m->uart_fifo.s;

	m->ppi_c = 0x01;

	m->kb_fifo.head = 0;
	m->kb_fifo.tail = sizeof(m->kb_fifo.buf) >> m->kb_fifo.s;

	m->cf_status = 0;
}

void
audio_cb(void *userdata, uint8_t *stream, int len)
{
	Machine *m = userdata;

	for(; len > 0; len -= 2, stream += 2)
		*(int16_t *)stream = SNG_calc(m->sng);
}

enum{
	FDS_CPU,
	FDS_PTY,
	FDS_SDL,
	NFDS
};

int
main(int argc, char *argv[])
{
	i8080 cpu;
	Machine	machine;
	Machine *m;
	int romfd, cffd, ret, pitch, x, y, buttonid;
	struct pollfd fds[NFDS];
	uint64_t val;
	struct itimerspec it;
	SDL_Window *window;
	SDL_Renderer *renderer;
	SDL_RendererInfo info;
	Uint32 format, p0, p1, p2;
	SDL_PixelFormat *pixelformat;
	SDL_Texture *texture;
	SDL_Event event;
	Uint32 *pixels;
	uint8_t *plane, *src;
	uint8_t b;
	SDL_MessageBoxData messageboxdata;
	SDL_MessageBoxButtonData buttons[3];
	SDL_AudioSpec want = {0}, have;
	SDL_AudioDeviceID audiodev;
	SDL_Joystick *js;
	SDL_JoystickGUID guid;

	if(argc < 3){
		fprintf(stderr, "usage: %s romfile cffile\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	m = &machine;

	i8080_init(&cpu);
	cpu.read_byte = read_byte;
	cpu.write_byte = write_byte;
	cpu.port_in = port_in;
	cpu.port_out = port_out;
	cpu.userdata = m;

	m->ram = malloc(256 * 1024);
	if(m->ram == NULL){
		perror("malloc()");
		exit(EXIT_FAILURE);
	}

	romfd = open(argv[1], O_RDONLY);
	if(romfd < 0){
		perror(argv[1]);
		exit(EXIT_FAILURE);
	}
	m->rom = mmap(NULL, 16 * 1024, PROT_READ, MAP_PRIVATE, romfd, 0);
	if(m->rom == MAP_FAILED){
		perror("mmap()");
		exit(EXIT_FAILURE);
	}

	m->ppi_a = 0xff;
	m->ppi_b = 0xff;

	m->kb_fifo.s = 2;
	m->uart_fifo.s = 0;

	m->js_buttons = 0;
	m->js_state = 0;
	m->js_timer = 0;

	reset(m);

	cffd = open(argv[2], O_RDWR);
	if(cffd < 0){
		perror(argv[2]);
		exit(EXIT_FAILURE);
	}
	m->cf_size = lseek(cffd, 0, SEEK_END);
	lseek(cffd, 0, SEEK_SET);
	m->cf_data = mmap(NULL, m->cf_size, PROT_READ | PROT_WRITE, MAP_SHARED, cffd, 0);
	if(m->cf_data == MAP_FAILED){
		perror("mmap()");
		exit(EXIT_FAILURE);
	}

	fds[FDS_CPU].fd = timerfd_create(CLOCK_MONOTONIC, 0);
	fds[FDS_CPU].events = POLLIN;
	it.it_interval.tv_sec = 0;
	it.it_interval.tv_nsec = 320000;
	it.it_value.tv_sec = 0;
	it.it_value.tv_nsec = 320000;
	timerfd_settime(fds[FDS_CPU].fd, 0, &it, NULL);

	fds[FDS_PTY].fd = posix_openpt(O_RDWR | O_NOCTTY);
	fds[FDS_PTY].events = POLLIN;
	unlockpt(fds[FDS_PTY].fd);
	puts(ptsname(fds[FDS_PTY].fd));

	if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_JOYSTICK) != 0){
		SDL_Log("SDL_Init(): %s", SDL_GetError());
		exit(EXIT_FAILURE);
	}
	window = SDL_CreateWindow("pac80emu", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 640, 480, 0);
	if(window == NULL){
		SDL_Log("SDL_CreateWindow(): %s", SDL_GetError());
		exit(EXIT_FAILURE);
	}
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if(renderer == NULL){
		SDL_Log("SDL_CreateRenderer(): %s", SDL_GetError());
		exit(EXIT_FAILURE);
	}
	SDL_RenderSetLogicalSize(renderer, 320, 240);
	SDL_RenderSetIntegerScale(renderer, SDL_TRUE);
	SDL_GetRendererInfo(renderer, &info);
	format = info.texture_formats[0];
	pixelformat = SDL_AllocFormat(format);
	p0 = SDL_MapRGB(pixelformat, 0, 0, 0);
	p1 = SDL_MapRGB(pixelformat, 42, 84, 126);
	p2 = SDL_MapRGB(pixelformat, 210, 168, 126);
	SDL_FreeFormat(pixelformat);
	texture = SDL_CreateTexture(renderer, format, SDL_TEXTUREACCESS_STREAMING, 320, 240);
	if(texture == NULL){
		SDL_Log("SDL_CreateTexture(): %s", SDL_GetError());
		exit(EXIT_FAILURE);
	}

	want.freq = 44100;
	want.format = AUDIO_S16SYS;
	want.channels = 1;
	want.samples = 128;
	want.callback = audio_cb;
	want.userdata = m;
	audiodev = SDL_OpenAudioDevice(NULL, 0, &want, &have, SDL_AUDIO_ALLOW_FREQUENCY_CHANGE);
	if(audiodev == 0){
		SDL_Log("SDL_OpenAudioDevice(): %s", SDL_GetError());
		exit(EXIT_FAILURE);
	}
	m->sng = SNG_new(3146875, have.freq);
	if(m->sng == NULL){
		perror("SNG_new()");
		exit(EXIT_FAILURE);
	}
	SNG_set_quality(m->sng, 0);
	SDL_PauseAudioDevice(audiodev, 0);

	js = NULL;

	fds[FDS_SDL].fd = timerfd_create(CLOCK_MONOTONIC, 0);
	fds[FDS_SDL].events = POLLIN;
	it.it_interval.tv_sec = 0;
	it.it_interval.tv_nsec = 16666666;
	it.it_value.tv_sec = 0;
	it.it_value.tv_nsec = 16666666;
	timerfd_settime(fds[FDS_SDL].fd, 0, &it, NULL);

	for(;;){
		ret = poll(fds, NFDS, -1);
		if(ret < 0 && errno != EINTR){
			perror("poll()");
			exit(EXIT_FAILURE);
		}

		if(fds[FDS_CPU].revents & POLLIN){
			ret = read(fds[FDS_CPU].fd, &val, sizeof(val));
			while(val-- > 0){
				while(cpu.cyc < 1007){
					if(cpu.iff && (m->ppi_c & (KINT | VINT | UINT)))
						i8080_interrupt(&cpu, 0xff);
					i8080_step(&cpu);
					if(cpu.halted){
						cpu.cyc = 1007;
						break;
					}
				}
				cpu.cyc -= 1007;
				if(!(m->ppi_c & KIBF) && fifo_count(&m->kb_fifo)){
					m->ppi_a = fifo_pop(&m->kb_fifo);
					m->ppi_c |= KIBF;
					if(m->ppi_c & KINTE)
						m->ppi_c |= KINT;
				}
				m->js_timer++;
				if(m->js_timer == 5){
					m->js_timer = 0;
					m->js_state = 0;
				}
			}
		}

		if(fds[FDS_PTY].revents & (POLLERR | POLLHUP)){
			close(fds[FDS_PTY].fd);
			fds[FDS_PTY].fd = posix_openpt(O_RDWR | O_NOCTTY);
			unlockpt(fds[FDS_PTY].fd);
			puts(ptsname(fds[FDS_PTY].fd));
		}

		if((m->uart_status & TXRDY) == 0){
			ret = write(fds[FDS_PTY].fd, &m->uart_tx, 1);
			m->uart_status |= TXRDY;
		}

		if(fds[FDS_PTY].revents & POLLIN){
			ret = read(fds[FDS_PTY].fd, &b, 1);
			if(ret > 0){
				fifo_push(&m->uart_fifo, b);
				if((m->uart_status & RXRDY) == 0){
					m->uart_rx = fifo_pop(&m->uart_fifo);
					m->uart_status |= RXRDY;
					if(m->ppi_c & UINTE)
						m->ppi_c |= UINT;
				}
			}
		}

		if(fds[FDS_SDL].revents & POLLIN){
			ret = read(fds[FDS_SDL].fd, &val, sizeof(val));

			if(m->ppi_c & VINTE)
				m->ppi_c |= VINT;

			while(SDL_PollEvent(&event)){
				if(event.type == SDL_KEYDOWN){
					fifo_push(&m->kb_fifo, xlat[event.key.keysym.scancode]);
				}else if(event.type == SDL_KEYUP){
					fifo_push(&m->kb_fifo, xlat[event.key.keysym.scancode] | 0x80);
				}else if(event.type == SDL_JOYBUTTONDOWN){
					if(event.jbutton.button < sizeof(js_map) / sizeof(js_map[0]))
						 m->js_buttons |= js_map[event.jbutton.button];
				}else if(event.type == SDL_JOYBUTTONUP){
					if(event.jbutton.button < sizeof(js_map) / sizeof(js_map[0]))
						 m->js_buttons &= ~js_map[event.jbutton.button];
				}else if(event.type == SDL_JOYHATMOTION){
					m->js_buttons &= ~(BUTTON_U | BUTTON_D | BUTTON_L | BUTTON_R);
					if(event.jhat.value & SDL_HAT_UP)
						 m->js_buttons |= BUTTON_U;
					if(event.jhat.value & SDL_HAT_DOWN)
						 m->js_buttons |= BUTTON_D;
					if(event.jhat.value & SDL_HAT_LEFT)
						 m->js_buttons |= BUTTON_L;
					if(event.jhat.value & SDL_HAT_RIGHT)
						 m->js_buttons |= BUTTON_R;
				}else if(event.type == SDL_JOYDEVICEADDED && js == NULL){
					guid = SDL_JoystickGetDeviceGUID(event.jdevice.which);
					if(memcmp(js_guid, &guid, sizeof(js_guid)) == 0){
						js = SDL_JoystickOpen(event.jdevice.which);
						if(js == NULL)
							SDL_Log("SDL_JoystickOpen(): %s", SDL_GetError());
					}
				}else if(event.type == SDL_JOYDEVICEREMOVED && js != NULL){
					SDL_JoystickClose(js);
					js = NULL;
				}else if(event.type == SDL_QUIT){
					messageboxdata.flags = 0;
					messageboxdata.window = NULL;
					messageboxdata.title = "Dialog";
					messageboxdata.message = "Leave?";
					messageboxdata.numbuttons = 3;
					buttons[0].flags = SDL_MESSAGEBOX_BUTTON_RETURNKEY_DEFAULT;
					buttons[0].buttonid = 0;
					buttons[0].text = "Quit";
					buttons[1].flags = 0;
					buttons[1].buttonid = 1;
					buttons[1].text = "Reset";
					buttons[2].flags = SDL_MESSAGEBOX_BUTTON_ESCAPEKEY_DEFAULT;
					buttons[2].buttonid = 2;
					buttons[2].text = "Cancel";
					messageboxdata.buttons = buttons;
					messageboxdata.colorScheme = NULL;
					ret = SDL_ShowMessageBox(&messageboxdata, &buttonid);
					if(ret != 0 || buttonid == 0){
						break;
					}else if(buttonid == 1){
						reset(m);
						cpu.pc = 0;
						cpu.iff = 0;
						cpu.halted = 0;
						cpu.interrupt_pending = 0;
					}
				}
			}
			if(event.type == SDL_QUIT && buttonid == 0)
				break;

			SDL_LockTexture(texture, NULL, (void **)&pixels, &pitch);
			plane = m->ram + ((m->ppi_c & VA15) ? 0x19810 : 0x11810);
			for(y = 0; y < 240; y++)
				for(x = 0, src = plane + y; x < 320; x += 8, src += 0x100)
					for(b = 0x80; b != 0; b >>= 1)
						*pixels++ = (*src & b) ? p1 : p0;
			SDL_UnlockTexture(texture);
			SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_NONE);
			SDL_RenderCopy(renderer, texture, NULL, NULL);

			SDL_LockTexture(texture, NULL, (void **)&pixels, &pitch);
			plane = m->ram + ((m->ppi_c & VA15) ? 0x1d810 : 0x15810);
			for(y = 0; y < 240; y++)
				for(x = 0, src = plane + y; x < 320; x += 8, src += 0x100)
					for(b = 0x80; b != 0; b >>= 1)
						*pixels++ = (*src & b) ? p2 : p0;
			SDL_UnlockTexture(texture);
			SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_ADD);
			SDL_RenderCopy(renderer, texture, NULL, NULL);
			SDL_RenderPresent(renderer);
		}
	}
	if(js)
		SDL_JoystickClose(js);
	SDL_CloseAudioDevice(audiodev);
	SNG_delete(m->sng);
	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
	return 0;
}
