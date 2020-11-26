/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:            MemoryBlock

Description:    Lite weight class for a CPU memory buffer that can be aligned
                and has proper move and assignment semantics.  Similar to
                STL std::array except unlike std::array<T, size_t> the size
                is not a template component so it can be defined at run time.
                
                If you need reference counting, use 
                std::shared_ptr<MemoryBLock<T>> mem = std::make_shared<MemoryBLock<T>>(size)

                MemoryBlock<T> has limited resizing to Append or Merge another 
                MemoryBlock<T> Append includes operator overloading of + and +=

Remarks:        This class differs from std::vector<T> in that it supports strides
                that we will be using to work with vertex buffers of dynamic
                structures.  Down side is you don't get the standard's algorithims
                to sort and find info in the vector.  This class is simple
                and lite weight and is intended wrap a memory pointer and not evolve
                into std::vector<T>.  Included is read/write from/to files in binary,
                raw binary, and text.  You may find it convienent to define data
                into std:vector<T> and then merge that into MemoryBlock<uint8_t>
                for storage within our engine (we make heavy use of this). An advatage
                of this is file read/write code in one place (here) and extension 
                into our memory pool class for the engine later on is simple since
                it builds upone this class.

Contact:        ChrisKing340@gmail.com

(c) Copyrighted 2017 Christopher H. King all rights reserved.

References:     1. https://msdn.microsoft.com/en-us/library/dd293665.aspx
                2. Stroustrup, Bjarne. A Tour of C++. Addison-Wesley, 2014

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#pragma once

#include <assert.h>
#include <fstream>
#include <string>
#include <emmintrin.h>

namespace King 
{
template<typename T, std::size_t align = 16> // bytes
class alignas(align) MemoryBlock
{
    /* variables */
private:
    size_t    _length = 0; // number of T elements in data allocation
    size_t    _stride = 1; // number of T elements to skip per operator [] indexing (convenient)
    T* _data = nullptr; // pointer is a multiple of alignment and only alligned if T new operator is also aligned

    /* methods */
public:
    // construction/life cycle
    explicit MemoryBlock() {}
    MemoryBlock(nullptr_t) : MemoryBlock() {} // allow nullptr construct since that is the default
    explicit MemoryBlock(size_t elementCount, size_t byteStridePerElement=1) : _length(elementCount*byteStridePerElement), _stride(byteStridePerElement), _data(new T[elementCount*byteStridePerElement]) {}
    explicit MemoryBlock(const MemoryBlock& other) : _length(other._length), _data(new T[other._length]), _stride(other._stride) { std::copy(other._data, other._data + _length, _data); } // Copy constructor
    explicit MemoryBlock(MemoryBlock&& other) noexcept : _data(nullptr), _length(0) { *this = std::move(other); } // Move constructor
    MemoryBlock(std::initializer_list<T> il) : _length(il.size()), _data(new T[_length]) { std::copy(std::begin(il), std::end(il), _data); }
    
    ~MemoryBlock() { Destroy(); }

    // operators
    void * operator new (size_t size) { return _aligned_malloc(size, align); }
    void   operator delete (void *p) { _aligned_free(static_cast<MemoryBlock*>(p)); }
    
    T operator [](size_t i) const { assert(i*_stride<_length); return _data[i*_stride]; } // accessor 
    T& operator [](size_t i) { assert(i*_stride<_length); return _data[i*_stride]; } // assignment
    
    T* operator->() { return _data; } // data pointer access
    const T* operator->() const { return _data; } // const data pointer access

    explicit operator bool() const { return (bool)_length; } // valid
    bool operator !() const { return !(bool)_data || !(bool)_length; } // invalid

    // allow nullptr assignment (for clearing)
    MemoryBlock& operator = (nullptr_t) 
    {
        if (_data) 
        {
            delete[] _data;
            _data = nullptr;
        }
        _length = 0;
        _stride = 1;
        return *this;
    }
    // copy assignment
    MemoryBlock & operator=(const T* other)
    {
        assert(_length > 0); // should be pre-initialized
        std::copy(other, other + _length, _data);
        return *this;
    }
    // copy assignment
    MemoryBlock & operator=(const MemoryBlock & other)
    {
        if (this != &other)
        {
            delete[] _data;

            _stride = other._stride;
            _length = other._length;
            _data = new T[_length]; // only aligned if T's new operator also aligns

            // SIMD not working with index buffer (it is uint32_t, is that a problem??)
            /*if ((0 == ((size_t)_data & (16 - 1))) && (0 == ((size_t)other._data & (16 - 1))))
                SIMDMemCopy(_data, other._data, _length);
            else*/
                std::copy(other._data, other._data + _length, _data);
        }
        return *this;
    }
    // move assignment
    MemoryBlock & operator=(MemoryBlock&& other)
    {
        if (this != &other)
        {
            delete[] _data;

            _data = other._data;
            _length = other._length;
            _stride = other._stride;

            other._data = nullptr;
            other._length = 0;
            other._stride = 1;
        }
        return *this;
    }
    MemoryBlock& operator+=(const MemoryBlock& other)
    {
        Append(other);
        return *this;
    }
    MemoryBlock operator+(const MemoryBlock& other)
    {
        MemoryBlock add(*this);
        add.Append(other);
        return add; // compiler should optimize and std::move
    }
    // Functionality
    inline void     Initialize(size_t lengthIn) { Destroy(); _length = lengthIn; _data = new T[_length]; }
    inline void     Destroy() { if (_data != nullptr) { delete[] _data;    _data = nullptr; _length = 0; _stride = 1; } }
    inline void     Fill(T valueIn) { for (size_t i = 0; i < _length; ++i) _data[i] = valueIn; }
    inline void     Merge(MemoryBlock<T> & other) // destroys other after 
                    {
                        if (this != &other && other._length > 0)
                        {
                            Append(other); // have to copy not move since we want a larger, continuous memory block
                            other.Destroy();
                        }
                    }
    inline void     Append(const MemoryBlock<T> & other)
                    {
                        if (this != &other && other._length > 0)
                        {
                            auto length = _length + other._length;
                            T* data = new T[length]; // only aligned if T's new operator also aligns

                            if(_length > 0)
                                std::copy(_data, _data + _length, data);
                            if(other._length > 0)
                                std::copy(other._data, other._data + other._length, data + _length);

                            Destroy();

                            _length = length;
                            _data = data;
                            _stride = other._stride;
                        }
                    }
    inline bool     ReadMemoryBlock(std::ifstream &dataFileIn) // Memory block binary (length coded first, then stride, data last)
                    {
                        if (!dataFileIn.is_open()) return false;

                        size_t readLength = _length;
                        dataFileIn.read(reinterpret_cast<char*>(&readLength), sizeof(size_t));
                        dataFileIn.read(reinterpret_cast<char*>(&_stride), sizeof(size_t));
                        if (dataFileIn.fail()) return false;
                    
                        if(readLength != _length) Initialize(readLength);
                    
                        dataFileIn.read(reinterpret_cast<char*>(_data), readLength * sizeof(T));

                        if (dataFileIn.fail()) return false;
                        return true;
                    }
    inline bool     WriteMemoryBlock(std::ofstream &outfileIn) // Memory block binary (length coded first, then stride, data last)
                    {
                        if (!outfileIn.is_open()) return false;
                        outfileIn.write(reinterpret_cast<const char *>(&_length), sizeof(size_t));
                        outfileIn.write(reinterpret_cast<const char*>(&_stride), sizeof(size_t));
                        outfileIn.write(reinterpret_cast<const char*>(_data), _length * sizeof(T));
                        if (outfileIn.fail()) return false;
                        return true;
                    }
    inline bool     ReadRawBinary(std::string & fileNameIn) // data only, no length or strided.  Reads entire contents of file
                    {
                        std::ifstream infile(fileNameIn, std::ifstream::binary);
                        if (infile.fail()) return false;
                        // get size of file
                        infile.seekg(0, infile.end);
                        auto size = infile.tellg();
                        infile.seekg(0);
        
                        Initialize(size);
                        infile.read(_data, _length);
                        infile.close();
                        if (infile.fail()) return false;
                        return true;
                    }
    inline bool     WriteRawBinary(std::string fileNameIn) // data only, no length or strided
                    {
                        std::ofstream outfile(fileNameIn, std::ofstream::binary | std::ofstream::trunc);
                        outfile.write(_data, _length);
                        outfile.close();
                        if (outfile.fail()) return false;
                        else return true;
                    }
    inline bool     WriteText(std::string fileNameIn) // readable format
                    {
                        std::ofstream outfile(fileNameIn, std::ofstream::trunc);
                        outfile << "Length: " << std::to_string(static_cast<T>(_length)) << '\n';
                        outfile << "Stride: " << std::to_string(static_cast<T>(_stride)) << '\n';
                        outfile << "Data:" << '\n';

                        for (size_t i = 0; i < _length; ++i)
                        {
                            outfile << std::to_string(static_cast<T>(_data[i])) << "\t";
                            if (i % 10 == 9) // every 10 values, new line
                                outfile << '\n';
                        }
                        outfile.close();
                        if (outfile.fail()) return false;
                        else return true;
                    }
    auto            Size() { return _length / _stride; }
    // Accessors
    T&              Get(size_t i) { assert(i*_stride < _length); return _data[i*_stride]; }
    T&              GetData() { return *_data; }
    const auto &    GetLength() const { return _length; } // number of T data
    const auto &    GetStride() const { return _stride; } // number of T elements to skip with operator[] indexing
    const auto      GetElements() const { return _length / _stride; } // number of stride elements
    // Assignments
    void            SetStride(const size_t & strideIn) { _stride = strideIn; }
    void            SetElementToValueByByteStride(const size_t & elementNumber, T* dataIn) 
                    {
                        T* dest = &Get(elementNumber);
                        std::copy(dataIn, dataIn + _stride, dest);
                    }

    // reference Utility.h from Microsoft open source miniEngine project
    inline void     SIMDMemCopy(void* __restrict _DestAligned16, const void* __restrict _SourceAligned16, size_t bufferSize)
    {
        assert (0 == ((size_t)_DestAligned16 & (16 - 1)));
        assert (0 == ((size_t)_SourceAligned16 & (16 - 1)));

        size_t NumQuadwords = (bufferSize + 16 - 1) / 16;

        __m128i* __restrict Dest = (__m128i * __restrict)_DestAligned16;
        const __m128i* __restrict Source = (const __m128i * __restrict)_SourceAligned16;

        // Discover how many quadwords precede a cache line boundary.  Copy them separately.
        size_t InitialQuadwordCount = (4 - ((size_t)Source >> 4) & 3) & 3;
        if (InitialQuadwordCount > NumQuadwords)
            InitialQuadwordCount = NumQuadwords;

        switch (InitialQuadwordCount)
        {
        case 3: _mm_stream_si128(Dest + 2, _mm_load_si128(Source + 2));     // Fall through
        case 2: _mm_stream_si128(Dest + 1, _mm_load_si128(Source + 1));     // Fall through
        case 1: _mm_stream_si128(Dest + 0, _mm_load_si128(Source + 0));     // Fall through
        default:
            break;
        }

        if (NumQuadwords == InitialQuadwordCount)
            return;

        Dest += InitialQuadwordCount;
        Source += InitialQuadwordCount;
        NumQuadwords -= InitialQuadwordCount;

        size_t CacheLines = NumQuadwords >> 2;

        switch (CacheLines)
        {
        default:
        case 10: _mm_prefetch((char*)(Source + 36), _MM_HINT_NTA);    // Fall through
        case 9:  _mm_prefetch((char*)(Source + 32), _MM_HINT_NTA);    // Fall through
        case 8:  _mm_prefetch((char*)(Source + 28), _MM_HINT_NTA);    // Fall through
        case 7:  _mm_prefetch((char*)(Source + 24), _MM_HINT_NTA);    // Fall through
        case 6:  _mm_prefetch((char*)(Source + 20), _MM_HINT_NTA);    // Fall through
        case 5:  _mm_prefetch((char*)(Source + 16), _MM_HINT_NTA);    // Fall through
        case 4:  _mm_prefetch((char*)(Source + 12), _MM_HINT_NTA);    // Fall through
        case 3:  _mm_prefetch((char*)(Source + 8), _MM_HINT_NTA);    // Fall through
        case 2:  _mm_prefetch((char*)(Source + 4), _MM_HINT_NTA);    // Fall through
        case 1:  _mm_prefetch((char*)(Source + 0), _MM_HINT_NTA);    // Fall through

            // Do four quadwords per loop to minimize stalls.
            for (size_t i = CacheLines; i > 0; --i)
            {
                // If this is a large copy, start prefetching future cache lines.  This also prefetches the
                // trailing quadwords that are not part of a whole cache line.
                if (i >= 10)
                    _mm_prefetch((char*)(Source + 40), _MM_HINT_NTA);

                _mm_stream_si128(Dest + 0, _mm_load_si128(Source + 0));
                _mm_stream_si128(Dest + 1, _mm_load_si128(Source + 1));
                _mm_stream_si128(Dest + 2, _mm_load_si128(Source + 2));
                _mm_stream_si128(Dest + 3, _mm_load_si128(Source + 3));

                Dest += 4;
                Source += 4;
            }

        case 0:    // No whole cache lines to read
            break;
        }

        // Copy the remaining quadwords
        switch (NumQuadwords & 3)
        {
        case 3: _mm_stream_si128(Dest + 2, _mm_load_si128(Source + 2));     // Fall through
        case 2: _mm_stream_si128(Dest + 1, _mm_load_si128(Source + 1));     // Fall through
        case 1: _mm_stream_si128(Dest + 0, _mm_load_si128(Source + 0));     // Fall through
        default:
            break;
        }

        _mm_sfence();
    }

    void SIMDMemFill(void* __restrict _DestAligned16, __m128 FillVector, size_t bufferSize)
    {
        assert(0 == ((size_t)_DestAligned16 & (16 - 1)));

        size_t NumQuadwords = (bufferSize + 16 - 1) / 16;

        register const __m128i Source = _mm_castps_si128(FillVector);
        __m128i* __restrict Dest = (__m128i * __restrict)_DestAligned16;

        switch (((size_t)Dest >> 4) & 3)
        {
        case 1: _mm_stream_si128(Dest++, Source); --NumQuadwords;     // Fall through
        case 2: _mm_stream_si128(Dest++, Source); --NumQuadwords;     // Fall through
        case 3: _mm_stream_si128(Dest++, Source); --NumQuadwords;     // Fall through
        default:
            break;
        }

        size_t WholeCacheLines = NumQuadwords >> 2;

        // Do four quadwords per loop to minimize stalls.
        while (WholeCacheLines--)
        {
            _mm_stream_si128(Dest++, Source);
            _mm_stream_si128(Dest++, Source);
            _mm_stream_si128(Dest++, Source);
            _mm_stream_si128(Dest++, Source);
        }

        // Copy the remaining quadwords
        switch (NumQuadwords & 3)
        {
        case 3: _mm_stream_si128(Dest++, Source);     // Fall through
        case 2: _mm_stream_si128(Dest++, Source);     // Fall through
        case 1: _mm_stream_si128(Dest++, Source);     // Fall through
        default:
            break;
        }

        _mm_sfence();
    }
};


}