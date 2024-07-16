#pragma once
#include <cassert>

// FIFO helper using circular buffer

template<class T, int size>
class FIFO final
{
public:
    void push(T val)
    {
        assert(!fullFlag);

        data[writeOff++] = val;

        if(writeOff == size)
            writeOff = 0;

        fullFlag = readOff == writeOff;
    }

    void pushIfNotFull(T val)
    {
        if(fullFlag)
            return;

        push(val);
    }

    T pop()
    {
        assert(!empty());

        auto ret = data[readOff++];

        if(readOff == size)
            readOff = 0;

        fullFlag = false;

        return ret;
    }

    T popOrDefault()
    {
        if(empty())
            return T{};

        return pop();
    }

    bool empty() const
    {
        return !fullFlag && readOff == writeOff;
    }

    bool full() const {return fullFlag;}

    int getCount() const
    {
        if(fullFlag)
            return size;

        if(writeOff >= readOff)
            return writeOff - readOff;

        return writeOff + size - readOff;
    }

private:
    T data[size];

    int readOff = 0, writeOff = 0;
    bool fullFlag = false;
};