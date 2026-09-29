#pragma once

class Camera {
    public: 
        virtual ~Camera() = default;
        virtual bool open() = 0;
        virtual bool read(Frame& out) = 0;
        virtual void close() = 0;
        virtual std::string name() const = 0;
};