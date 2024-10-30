#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <unistd.h>

#include <SDL2/SDL.h>

#include "8080/i8080.h"

typedef struct Machine Machine;
struct Machine{
	uint8_t *ram;
	uint8_t *rom;
	uint8_t *map[4];
	uint8_t uart_rx;
	uint8_t uart_tx;
	uint8_t uart_status;
	uint16_t cf_scount;
	uint16_t cf_bcount;
	uint32_t cf_lba;
	uint8_t cf_status;
	uint8_t *cf_data;
	off_t cf_size;
	uint8_t ppi_a;
	uint8_t ppi_b;
	uint8_t ppi_c;
	uint8_t kb_buf[64];
	uint8_t kb_head;
	uint8_t kb_tail;
};

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
			m->uart_status &= ~2;
			return m->uart_rx;
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
			m->ppi_c &= ~(1 << 5);
			return m->ppi_a;
		case 1: /* port b */
			return m->ppi_b;
		case 4: /* port c */
			return m->ppi_c;
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
			m->uart_status &= ~1;
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
	case 0x18:	/* PPI */
		switch(port & 5){
		case 0: /* port a */
		case 1: /* port b */
			break;
		case 4: /* port c */
			m->ppi_c = (m->ppi_c & 0xe8) | (val & 0x17);
			break;
		case 5: /* control */
			if((val & 0x80) == 0){
				bit = 1 << ((val >> 1) & 7);
				if(val & 1)
					val = m->ppi_c | bit;
				else
					val = m->ppi_c & ~bit;
				m->ppi_c = (m->ppi_c & 0xe8) | (val & 0x17);
			}
			break;
		}
	case 0x38:	/* PSG */
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

static inline uint8_t
kb_space(Machine *m)
{
	return m->kb_tail - m->kb_head;
}

static inline uint8_t
kb_count(Machine *m)
{
	return sizeof(m->kb_buf) - kb_space(m);
}

static inline void
kb_push(Machine *m, uint8_t data)
{
	if(kb_space(m))
		m->kb_buf[m->kb_head++ & 0x3f] = data;
}

static inline uint8_t
kb_pop(Machine *m)
{
	return m->kb_buf[m->kb_tail++ & 0x3f];
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
	int romfd, cffd, ret, pitch, x, y;
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

	m->map[0] = m->rom;
	m->map[1] = m->rom;
	m->map[2] = m->rom;
	m->map[3] = m->rom;

	m->uart_status = 1;

	m->ppi_a = 0xff;
	m->ppi_b = 0xff;
	m->ppi_c = 0;

	m->kb_head = 0;
	m->kb_tail = sizeof(m->kb_buf);

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

	if(SDL_Init(SDL_INIT_VIDEO) != 0){
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
			while(cpu.cyc < 1007)
				i8080_step(&cpu);
			cpu.cyc -= 1007;
			if(!(m->ppi_c & (1 << 5)) && kb_count(m)){
				m->ppi_a = kb_pop(m);
				m->ppi_c |= (1 << 5);
			}
		}

		if(fds[FDS_PTY].revents & (POLLERR | POLLHUP)){
			close(fds[FDS_PTY].fd);
			fds[FDS_PTY].fd = posix_openpt(O_RDWR | O_NOCTTY);
			unlockpt(fds[FDS_PTY].fd);
			puts(ptsname(fds[FDS_PTY].fd));
		}

		if((m->uart_status & 1) == 0){
			ret = write(fds[FDS_PTY].fd, &m->uart_tx, 1);
			m->uart_status |= 1;
		}

		if(fds[FDS_PTY].revents & POLLIN){
			ret = read(fds[FDS_PTY].fd, &m->uart_rx, 1);
			if(ret > 0)
				m->uart_status |= 2;
		}

		if(fds[FDS_SDL].revents & POLLIN){
			ret = read(fds[FDS_SDL].fd, &val, sizeof(val));
			while(SDL_PollEvent(&event)){
				if(event.type == SDL_KEYDOWN || event.type == SDL_KEYUP)
					kb_push(m, xlat[event.key.keysym.scancode] | (~event.key.state << 7));
				else if(event.type == SDL_QUIT)
					break;
			}
			if(event.type == SDL_QUIT)
				break;

			SDL_LockTexture(texture, NULL, (void **)&pixels, &pitch);
			plane = m->ram + ((m->ppi_c & 1) ? 0x19810 : 0x11810);
			for(y = 0; y < 240; y++)
				for(x = 0, src = plane + y; x < 320; x += 8, src += 0x100)
					for(b = 0x80; b != 0; b >>= 1)
						*pixels++ = (*src & b) ? p1 : p0;
			SDL_UnlockTexture(texture);
			SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_NONE);
			SDL_RenderCopy(renderer, texture, NULL, NULL);

			SDL_LockTexture(texture, NULL, (void **)&pixels, &pitch);
			plane = m->ram + ((m->ppi_c & 1) ? 0x1d810 : 0x15810);
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
	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
	return 0;
}
