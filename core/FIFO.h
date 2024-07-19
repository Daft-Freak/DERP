#pragma once
#include <cassert>

// FIFO helper using circular buffer

template<class T, int size>
class FIFO final
{
public:
    bool push(T val)
    {
        assert(!fullFlag);

        data[writeOff] = val;

        writeOff = (writeOff + 1) % size;

        fullFlag = readOff == writeOff;
        return fullFlag;
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

        auto ret = data[readOff];

        readOff = (readOff + 1) % size;

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
        return readOff == writeOff && !fullFlag;
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

    unsigned int readOff = 0, writeOff = 0;
    bool fullFlag = false;
};