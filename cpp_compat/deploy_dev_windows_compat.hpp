#pragma once

#ifdef _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <cstddef>
#include <cstdint>
#include <process.h>
#include <sys/types.h>

#pragma comment(lib, "ws2_32.lib")

namespace deploy_dev_windows_compat {

struct WinsockRuntime {
    WinsockRuntime() noexcept {
        WSADATA data{};
        ready = WSAStartup(MAKEWORD(2, 2), &data) == 0;
    }

    ~WinsockRuntime() {
        if (ready) WSACleanup();
    }

    bool ready = false;
};

inline WinsockRuntime winsock_runtime;

}  // namespace deploy_dev_windows_compat

#ifndef F_GETFL
#define F_GETFL 3
#endif
#ifndef F_SETFL
#define F_SETFL 4
#endif
#ifndef O_NONBLOCK
#define O_NONBLOCK 0x0004
#endif

inline int fcntl(int descriptor, int operation, int flags = 0) noexcept {
    if (operation == F_GETFL) return 0;
    if (operation != F_SETFL) return -1;
    u_long nonblocking = (flags & O_NONBLOCK) != 0 ? 1UL : 0UL;
    return ioctlsocket(static_cast<SOCKET>(descriptor), FIONBIO, &nonblocking);
}

inline int close(int descriptor) noexcept {
    return closesocket(static_cast<SOCKET>(descriptor));
}

// debug_web is disabled by default on Windows. These fail-closed POSIX SHM
// shims let the publisher compile and report ready()==false instead of
// pretending that a Linux /dev/shm mapping exists.
#ifndef PROT_READ
#define PROT_READ 0x1
#endif
#ifndef PROT_WRITE
#define PROT_WRITE 0x2
#endif
#ifndef MAP_SHARED
#define MAP_SHARED 0x01
#endif
#ifndef MAP_FAILED
#define MAP_FAILED reinterpret_cast<void*>(static_cast<std::intptr_t>(-1))
#endif

inline int shm_open(const char*, int, int) noexcept { return -1; }
inline int ftruncate(int, off_t) noexcept { return -1; }
inline void* mmap(void*, std::size_t, int, int, int, off_t) noexcept {
    return MAP_FAILED;
}
inline int munmap(void*, std::size_t) noexcept { return 0; }

#ifndef getpid
#define getpid _getpid
#endif

#endif  // _WIN32
