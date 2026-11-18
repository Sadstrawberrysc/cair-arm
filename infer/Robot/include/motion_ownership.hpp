#pragma once
#include "realman_command.hpp"
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

// Host-local cooperative ownership shared by the production runtime, coarse
// MoveJ_P tool and Collision executor. Not a controller/distributed interlock.
class RMMotionOwnership {
public:
    RMMotionOwnership() = default;
    RMMotionOwnership(const RMMotionOwnership&) = delete;
    RMMotionOwnership& operator=(const RMMotionOwnership&) = delete;
    ~RMMotionOwnership() { if (fd_>=0) close(fd_); }
    bool Acquire(const RMConnectionConfig& connection) {
        if (fd_>=0 || connection.ip.empty() || connection.ip.find_first_not_of("0123456789.")!=std::string::npos)
            return false;
        const std::string path="/tmp/uspilot-motion-"+connection.ip+"-"+std::to_string(connection.port)+".lock";
        fd_=open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
        if (fd_<0) return false;
        if (flock(fd_,LOCK_EX|LOCK_NB)==0) return true;
        close(fd_); fd_=-1; return false;
    }
private:
    int fd_=-1;
};
