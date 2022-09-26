/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          MemoryBlock

Description:    Dynamic CPU memory buffer that can be aligned and has proper 
                move and copy assignment semantics. Supports file read and 
                write access to/from the block. Supports strides of the data
                type to work with vertex buffers of dynamic structures. 
                Supports custum data types by properly calling destructors on
                delete and constructors when emplace new. This class wraps a 
                memory pointer and tracks _length in use and _capacity 
                available. Methods to merge, append, or split MemoryBlocks and
                support of std::initializer_list<T>.

Remarks:        Header only implementation for inline speed and code reduction
                of unused methods.

Revisions:      7/23/2021: Implemented MinMax(...), Min(), and Max() for efficient determination of the data pool
                12/31/2021: Implemented thread safety with mutex locks on data writes
                1/1/2022: Write raw binary updated to work with any data type, previous was just T = sizeof(char).
                    Code cleaned up to be more readable and finalized the thread safety with addiontional mutex locks.
                6/26/2022: 1) Revised allocations and implemented Allocate(...) and revised Destroy() to properly handle 
                    alignment new and delete.
                    2) Implemented Reallocate() to enable moves rather than copy and also properly call 
                    contructors if the class is used for more than standard data types.
                    3) Implemented Dynamic resizing by adding PushBack(...), EmplaceBack(...), and PopBack()
                    4) Implemented Clear() that calls destructor of elements for _length, but not of allocation of _capacity.
                        This required refactoring of delete from [] operators to the ::operater delete which does not call
                        destructors when freeing and just the memory we specify.
                9/18/2022: Revised PushBack() dynamic memory allocator to both reduce reallocation frequency on small buffers 
                    and cap resizing growth rate above 256 to increments of 256. Also changed initial allocation to null
                    inlieu of two as the initial often was never enough and almost always results in a reallocation which is
                    costly.
                9/19/2022: Added file read and write for 8, 16, 32, and 64 bits
                9/25/2022: Improved exception safety for bad memory allocations. Some allowed to fail and continue, others throw
                    if it cannot continue. Constructors and Destructor do not throw. After Initializing a MemoryBlock, check 
                    GetLength() != 0 for successful data space.

Contact:        ChrisKing340@gmail.com

(c) Copyrighted 2017-2022 Christopher H. King all rights reserved with permissions as noted here.

References:     1. https://msdn.microsoft.com/en-us/library/dd293665.aspx
                2. Stroustrup, Bjarne. A Tour of C++. Addison-Wesley, 2014
                3. Williams, Anthony. C++ Concurrency IN ACTION, 2nd edition. Manning Publications, 2019

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
#include <iostream>
#include <new>
#include <fstream>
#include <string>
#include <mutex>
#include <exception>

namespace King 
{
template<typename T, std::size_t align = 16> // bytes
class alignas(align) MemoryBlock
{
    /* variables */
protected:
    size_t              _length = 0; // number of T elements assigned within data allocation
    size_t              _capacity = 0; //number of T elements reserved for in data allocation
    size_t              _stride = 1; // number of T elements to skip per operator [] indexing (convenient). Note Size() is now depreciated to make this clearer in loops
    T*                  _data = nullptr; // pointer is a multiple of alignment and only alligned if T new operator is also aligned
    mutable std::mutex  _mutex;
    /* methods */
public:
    // construction/life cycle
    explicit MemoryBlock() noexcept : _capacity(0) {} // Allocates empty, PushBack() handles dynamic increases
    MemoryBlock(nullptr_t) noexcept : MemoryBlock() {} // allow nullptr construct since that is the default
    explicit MemoryBlock(size_t elementCount, size_t byteStridePerElement = 1) noexcept : _stride(byteStridePerElement) { Allocate(elementCount * _stride); Fill(); }
    explicit MemoryBlock(const MemoryBlock& other) noexcept { *this = other; } // Copy constructor
    explicit MemoryBlock(MemoryBlock&& other) noexcept { *this = std::move(other); } // Move constructor
    MemoryBlock(std::initializer_list<T> il) noexcept : _length(il.size()) { Allocate(_length); if (_data) std::copy(std::begin(il), std::end(il), _data); }
    
    ~MemoryBlock() noexcept { Destroy(); }

    // operators
    void * operator new (size_t size) { return _aligned_malloc(size, align); }
    void   operator delete (void *p) { _aligned_free(static_cast<MemoryBlock*>(p)); }
    
    const T& operator [](size_t i) const { assert(i*_stride<_length); return _data[i*_stride]; } // accessor 
    T& operator [](size_t i) { assert(i*_stride<_capacity); return _data[i*_stride]; } // assignment
    
    explicit operator T* () const { return 0; } // prevents implicit conversions to other data types, use static_cast
    T* operator->() { return _data; } // data pointer access
    const T* operator->() const { return _data; } // const data pointer access
    explicit operator bool() const { return (bool)_length; } // valid
    bool operator !() const { return !(bool)_data || !(bool)_length; } // invalid
    // Appends will throw, original data lost
    inline MemoryBlock& operator+=(const MemoryBlock& other);
    inline MemoryBlock operator+(const MemoryBlock& other);
    // Assignment operators no not throw, original data lost, as the intent was to replace the data anyways
    inline MemoryBlock& operator=(const T* other) noexcept; // if fails, _length = 0, _capacity = 0, and _data = nullptr;
    inline MemoryBlock& operator=(const MemoryBlock& other) noexcept; // if fails, _length = 0, _capacity = 0, and _data = nullptr;
    inline MemoryBlock& operator=(MemoryBlock&& other) noexcept;

    // Functionality
    inline void                 Initialize(size_t lengthIn) noexcept { Allocate(lengthIn); Fill(); }
    inline void                 Reserve(size_t capacityIn) noexcept { ReAllocate(capacityIn); }
    inline void                 Destroy() noexcept;
    
    // dynamic allocations
    inline void                 PushBack(const T& valueIn);
    inline void                 PushBack(const T&& valueIn);
    template<typename... Args> inline T& EmplaceBack(Args&&... args);
    inline void                 PopBack();

    // block methods
    inline void                 Clear() noexcept; // calls destructor of each object
    inline void                 Fill(); // fill entire capacity with default contructor value of T type
    inline void                 Fill(T valueIn); // fill entire capacity with valueIn
    inline void                 Split(size_t elementIndex, MemoryBlock<T>* out); // *this range becomes [begin to elementIndex-1] and *out range becomes [elementIndex to end]
    inline void                 Merge(MemoryBlock<T>& other); // other is de-initalized after merge
    inline void                 Append(const MemoryBlock<T>& other); // copies other

    void                        Copy(const size_t& startElementNumber, const T* srcIn, size_t size = 0); // element is _length(T's) / _stride
    void                        MinMax(T* minOut, T* maxOut);
    T                           Min();
    T                           Max();

    [[deprecated("Use Read instead.")]]
    inline bool ReadMemoryBlock(std::ifstream& dataFileIn) {}
    [[deprecated("Use Write instead.")]]
    inline bool WriteMemoryBlock(std::ofstream& outfileIn) {}

    inline bool                 Read(std::ifstream& dataFileIn); // pass stream so we could pack multiple MemoryBlocks into one file
    inline bool                 Write(std::ofstream& outfileIn);

    inline bool                 ReadRawBinary(std::string& fileNameIn); // length unknown, so read entire file and use file length as byte size
    inline bool                 WriteRawBinary(std::string fileNameIn);

    inline bool                 WriteText(std::string fileNameIn); // for easy debugging of data

    [[deprecated("Use GetLength() or GetElements() depending on use instead.")]]
    auto Size() { return _length; } // note, use GetElements() for stride lengths
    // Accessors
    T&                          Get(size_t i) { assert(i * _stride < _length); return _data[i * _stride]; }
    const T&                    Get(size_t i) const { assert(i * _stride < _length); return _data[i * _stride]; }
    T*                          GetData() { return _data; }
    const T*                    GetData() const { return _data; }
    const auto&                 GetLength() const { return _length; } // number of T data
    const auto&                 GetCapacity() const { return _capacity; } // number of T data
    const auto&                 GetStride() const { return _stride; } // number of T elements to skip with operator[] indexing
    const auto                  GetByteSize() const { return _length * sizeof(T); }
    const auto                  GetElements() const { return _length / _stride; } // number of stride elements
    const auto&                 GetElement(size_t i) const { return operator [](i); }
    const auto&                 GetFront() const { assert(_length); return _data[0]; }
    const auto&                 GetBack() const { return operator [](GetElements() - 1); }
    // Assignments
    void                        Set(size_t i, T valueIn) { if (i * _stride < _length) _data[i * _stride] = valueIn; }
    void                        SetLength(const size_t& lengthIn);
    void                        SetStride(const size_t& strideIn) { std::lock_guard<std::mutex> guard(_mutex); _stride = strideIn; }
    // Memory helpers
    uint8_t                     Read1Byte(size_t elem);
    uint16_t                    Read2ByteWord(size_t elem);
    uint32_t                    Read4ByteDword(size_t elem);
    uint64_t                    Read8ByteDDword(size_t elem);
    float                       ReadFloat(size_t elem);

    void                        Write1Byte(size_t elem, uint8_t byteIn);
    void                        Write2ByteWord(size_t elem, uint16_t wordIn);
    void                        Write4ByteDword(size_t elem, uint32_t dwordIn);
    void                        Write8ByteDword(size_t elem, uint64_t ddwordIn);
    // File helpers
    uint8_t                     Read1Byte(std::ifstream& dataFileIn);
    uint16_t                    Read2ByteWord(std::ifstream& dataFileIn);
    uint32_t                    Read4ByteDword(std::ifstream& dataFileIn);
    uint64_t                    Read8ByteDDword(std::ifstream& dataFileIn);
    float                       ReadFloat(std::ifstream& dataFileIn);

    void                        Write1Byte(std::ofstream& dataFileOut, uint8_t byteIn);
    void                        Write2ByteWord(std::ofstream& dataFileOut, uint16_t wordIn);
    void                        Write4ByteDword(std::ofstream& dataFileOut, uint32_t dwordIn);
    void                        Write8ByteDDword(std::ofstream& dataFileOut, uint64_t ddwordIn);

private:
    void                        Allocate(const std::size_t capacityIn) noexcept; // if fails, _length = 0, _capacity = 0, and _data = nullptr;
    void                        ReAllocate(const std::size_t capacityIn) noexcept; // if fails, _length = 0, _capacity = 0, and _data = nullptr;
};

template<typename T, std::size_t align>
inline MemoryBlock<T, align>& MemoryBlock<T, align>::operator+=(const MemoryBlock& other)
{
    Append(other);
    return *this;
}
template<typename T, std::size_t align>
inline MemoryBlock<T, align> MemoryBlock<T, align>::operator+(const MemoryBlock& other)
{
    MemoryBlock add(*this);
    add.Append(other);
    return add; // compiler should optimize and std::move
}
// must already be initialized
template<typename T, std::size_t align>
inline MemoryBlock<T, align>& MemoryBlock<T, align>::operator=(const T* other) noexcept
{
    if (!_length)
        return *this;

    auto l = _length;
    Clear();

    // Emplace new
    for (size_t i = 0; i < l; ++i)
        new(_data + i) T(*(other + i));

    _length = l;

    return *this;
}
// copy assignment
template<typename T, std::size_t align>
inline MemoryBlock<T, align>& MemoryBlock<T, align>::operator=(const MemoryBlock& other) noexcept
{
    if (this != &other)
    {
        // keeps our _capacity if > other._length
        if (_capacity < other._length)
        {
            Destroy();
            Allocate(other._length);
        }

        // bad allocation? we cannot complete the copy
        if (!_data)
            return *this;

        std::lock_guard<std::mutex> guard(_mutex);
        // Clear then emplace new
        for (size_t i = 0; i < other.GetLength(); ++i)
        {
            _data[i].~T();
            new(&_data[i]) T(other._data[i]);
        }
        // Clear object beyond other._length
        for (size_t i = other.GetLength(); i < _length; ++i)
        {
            _data[i].~T();
        }

        _stride = other._stride;
        _length = other._length;
    }
    return *this;
}
// move assignment
template<typename T, std::size_t align>
inline MemoryBlock<T, align>& MemoryBlock<T, align>::operator=(MemoryBlock&& other) noexcept
{
    if (this != &other)
    {
        Destroy();

        std::lock_guard<std::mutex> guard(_mutex);

        _data = other._data;
        _length = other._length;
        _capacity = other._capacity;
        _stride = other._stride;
        // Reset the data pointer from the source object to null so that
        // the destructor does not call free of the memory multiple times.
        std::lock_guard<std::mutex> guardOther(other._mutex);
        other._data = nullptr;
        other._length = 0;
        other._capacity = 0;
        other._stride = 1;
    }
    return *this;
}

// Assignments
template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::SetLength(const size_t& lengthIn) 
{ 
    auto oldLength = _length;

    std::lock_guard<std::mutex> guard(_mutex);
    // shrinks?
    if (lengthIn < _length)
    {
        while (_length > lengthIn)
        {
            --_length;
            _data[_length].~T();
        }
    }
    // grows?
    else if (lengthIn > _capacity)
        ReAllocate(lengthIn);
    // bad ReAllocation?
    if (!_data)
        _length = 0;
    else
        _length = lengthIn; 

    if (_length > oldLength)
    {
        // construct emplace
        do 
        {
            new(&_data[oldLength]) T();
        } 
        while (++oldLength < _length);
    }
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Allocate(const std::size_t capacityIn) noexcept
{
    Destroy();
    std::lock_guard<std::mutex> guard(_mutex);
    _capacity = capacityIn;
    // only aligned if T's new operator also aligns
    try
    {
        _data = (T*)(::operator new (_capacity * sizeof(T)));
    }
    catch (const std::bad_alloc& e)
    {
        _capacity = 0;
        _length = 0;
        _data = nullptr;
    }

    if (_capacity < _length)
        _length = _capacity;
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::ReAllocate(const std::size_t capacityIn) noexcept
{
    if (_capacity == capacityIn)
    {
        return;
    }

    // only aligned if T's new operator also aligns
    T* data = nullptr;
    try
    {
        data = (T*)::operator new(capacityIn * sizeof(T));
    }
    catch (const std::bad_alloc& e)
    {
        _capacity = 0;
        _length = 0;
        data = nullptr;
    }

    // shrink?
    std::size_t newLength(_length);
    if (capacityIn < newLength)
    {
        newLength = capacityIn;
    }

    // emplace new, move to data
    {
        std::lock_guard<std::mutex> guard(_mutex);
        for (size_t i = 0; i < newLength; ++i)
            new (&data[i]) T(std::move(_data[i]));
    }

    // old data call destructors
    Clear();
    // delete without calling destructors
    std::lock_guard<std::mutex> guard(_mutex);
    ::operator delete(_data, _capacity * sizeof(T));

    _data = data;
    _length = newLength;
    _capacity = capacityIn;
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Destroy() noexcept
{ 
    if (_capacity) 
    { 
        // remove all elements
        Clear();

        std::lock_guard<std::mutex> guard(_mutex);
        // delete our memory reserve
        if (_data != nullptr)
            ::operator delete(_data, _capacity * sizeof(T));

        _data = nullptr; 
        _length = 0;
        _capacity = 0;
        //_stride = 1; leave this same as before Destroy so order of setting it does not matter. Often Destroy would be called implicitly
    } 
}
// add an element to the end
template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::PushBack(const T& valueIn)
{
    if (_length >= _capacity)
        ReAllocate(_capacity + _capacity / 2);

    _data[_length] = valueIn;
    ++_length;
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::PushBack(const T&& valueIn)
{
    // dynamic to 4, 8, 12, 18, 27, 40, 60, 90, 135... when starting from zero
    if (!_capacity)
        Allocate(4);
    if (_length >= _capacity)
    {
        if (_capacity >= 4 && _capacity < 16)
            ReAllocate(16);
        else if (_capacity > 256)
            ReAllocate((_capacity / 256 + 1) * 256);
        else
            ReAllocate(_capacity + _capacity / 2); // 16, 24, 36, 48, 72,...
    }

    _data[_length] = std::move(valueIn);
    ++_length;
}
// remove the last element
template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::PopBack()
{
    if (_length > 0)
    {
        --_length;
        _data[_length].~T();
    }
}

template<typename T, std::size_t align>
template<typename ...Args>
inline T& MemoryBlock<T, align>::EmplaceBack(Args && ...args)
{
    // dynamic to 4, 8, 12, 18, 27, 40, 60, 90, 135... when starting from zero
    if (!_capacity)
        Allocate(4);
    if (_length >= _capacity)
    {
        if (_capacity >= 4 && _capacity < 16)
            ReAllocate(16);
        else if (_capacity > 256)
            ReAllocate((_capacity / 256 + 1) * 256);
        else
            ReAllocate(_capacity + _capacity / 2); // 16, 24, 36, 48, 72,...
    }

    if (!_data)
    {
        // we cannot return a bad address
        throw std::bad_alloc("BAD MEMORY ALLOCATION in MemoryBlock::EmplaceBack(...)");
    }

    new(&_data[_length]) T(std::forward<Args>(args)...);
    return _data[_length++];
}
// remove all elements
template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Clear() noexcept
{
    std::lock_guard<std::mutex> guard(_mutex);
    while (_length > 0)
    {
        --_length;
        _data[_length].~T(); // all destructors of standard types are noexcept
    }
    _length = 0;
}

// Fill all allocated memory (size of _capacity)
template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Fill()
{
    // call destructors
    Clear();
    
    std::lock_guard<std::mutex> guard(_mutex);
    // use the default constructor values;
    // copy
    for (size_t i = 0; i < _length; ++i)
        _data[i] = T();
    // construct emplace
    for (size_t i = _length; i < _capacity; ++i)
        new(&_data[i]) T();

    _length = _capacity;
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Fill(T valueIn) 
{ 
    // call destructors
    Clear();

    std::lock_guard<std::mutex> guard(_mutex); 
    // copy
    for (size_t i = 0; i < _length; ++i) 
        _data[i] = valueIn;
    // construct emplace
    for (size_t i = _length; i < _capacity; ++i)
        new(&_data[i]) T(valueIn);

    _length = _capacity;
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Split(size_t elementIndex, MemoryBlock<T>* out)
{
    // keep left of split, place in *out right of split
    assert(elementIndex * _stride != _length);
    if (!elementIndex) return;
    assert(_data != nullptr);
    assert(elementIndex * _stride < _length);
    assert(out != nullptr);

    // define buffer lengths
    const size_t length1 = (GetElements() - elementIndex) * _stride;
    const size_t length2 = _length - length1;
    // copy the end of our buffer into the out buffer
    out->ReAllocate(length2);
    if (!_data)
    {
        throw std::bad_alloc();
    }
    std::copy(_data + length1, _data + _length, out->_data);
    // unnecessary, just for safety
    std::memset(_data + length1, 0, length2);
    out->SetStride(_stride);
    // shrink our buffer
    _length = length1;
    // note our capacity is unaffected, so memory is still reserved. If we need to reduce
    // memory, ReAllocate(_length) after the split.
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Merge(MemoryBlock<T>& other) // destroys other after 
{
    if (this != &other && other._length > 0)
    {
        Append(other); 
        other.Destroy();
    }
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Append(const MemoryBlock<T>& other) // by copy
{
    assert(_stride == other._stride);
    if (this != &other && other._length > 0)
    {
        const auto newLength = _length + other._length;
        if (newLength > _capacity)
            ReAllocate(newLength);

        if (!_data)
        {
            throw std::bad_alloc();
        }

        std::lock_guard<std::mutex> guard(_mutex);
        
        // emplace new
        for (size_t i = 0; i < other._length; ++i)
            new (_data + _length + i) T(other._data[i]);

        _length = newLength;
    }
}

template<typename T, std::size_t align>
inline bool MemoryBlock<T, align>::Read(std::ifstream& dataFileIn) // Memory block binary (length coded first, then stride, data last)
{
    if (!dataFileIn.is_open()) return false;
    if (!dataFileIn.good()) return false;

    size_t readLength;
    size_t readStride;

    dataFileIn.read(reinterpret_cast<char*>(&readLength), sizeof(size_t));
    dataFileIn.read(reinterpret_cast<char*>(&readStride), sizeof(size_t));

    if (dataFileIn.fail()) return false;

    if (readLength <= _capacity)
    {
        // re-initialize buffer
        Clear();
        // construct emplace
        for (size_t i = 0; i < readLength; ++i)
            new(&_data[i]) T();
        _length = readLength;
        assert(_length <= _capacity);
    }
    else
    {
        // expand
        Clear();
        ReAllocate(readLength);
        Fill();
    }

    std::lock_guard<std::mutex> guard(_mutex);
    dataFileIn.read(reinterpret_cast<char*>(_data), _length * sizeof(T));

    _stride = readStride;

    if (dataFileIn.fail()) return false;
    return true;
}

template<typename T, std::size_t align>
inline bool MemoryBlock<T, align>::Write(std::ofstream& outfileIn) // Memory block binary (length coded first, then stride, data last)
{
    if (!outfileIn.is_open()) return false;
    outfileIn.write(reinterpret_cast<const char*>(&_length), sizeof(size_t));
    outfileIn.write(reinterpret_cast<const char*>(&_stride), sizeof(size_t));
    outfileIn.write(reinterpret_cast<const char*>(_data), _length * sizeof(T));
    if (outfileIn.fail()) return false;
    return true;
}

template<typename T, std::size_t align>
inline bool MemoryBlock<T, align>::ReadRawBinary(std::string& fileNameIn) // data only, no stride.  Reads entire contents of file
{
    std::ifstream infile(fileNameIn, std::ifstream::binary);
    if (infile.fail()) return false;
    // get size of file
    infile.seekg(0, infile.end);
    int bytes = infile.tellg();
    infile.seekg(0);
    assert(bytes >= sizeof(T));

    auto readLength = bytes / sizeof(T);

    if (readLength <= _capacity)
    {
        // re-initialize buffer
        Clear();
        // construct emplace
        for (size_t i = 0; i < readLength; ++i)
            new(&_data[i]) T();
        _length = readLength;
        assert(_length <= _capacity);
    }
    else
    {
        // expand
        Clear();
        ReAllocate(readLength);
        Fill();
    }

    std::lock_guard<std::mutex> guard(_mutex);

    infile.read(reinterpret_cast<const char*>(_data), _length * sizeof(T));
    infile.close();
    if (infile.fail()) return false;
    return true;
}

template<typename T, std::size_t align>
inline bool MemoryBlock<T, align>::WriteRawBinary(std::string fileNameIn) // data only, no length or stride
{
    const auto& bytes = GetByteSize();

    if (bytes)
    {
        std::ofstream outfile(fileNameIn, std::ofstream::binary | std::ofstream::trunc);
        outfile.write(_data, bytes);
        outfile.close();
        if (outfile.fail()) return false;
        else return true;
    }
    return false;
}

template<typename T, std::size_t align>
inline bool MemoryBlock<T, align>::WriteText(std::string fileNameIn) // readable format
{
    std::ofstream outfile(fileNameIn, std::ofstream::trunc);
    outfile << "Length: " << std::to_string(_length) << '\n';
    outfile << "Stride: " << std::to_string(_stride) << '\n';
    outfile << "Elements: " << std::to_string(GetElements()) << '\n';
    outfile << "Bytes: " << std::to_string(GetByteSize()) << '\n';
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

//void King::from_json(const json& j, UIntPoint2& to) { j.at("x").get_to(to.u[0]); j.at("y").get_to(to.u[1]); }

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Copy(const size_t& startElementNumber, const T* srcIn, size_t size)
{
    if (!size) size = _stride; // 1 element by default
    T* dest = &Get(startElementNumber);
    std::lock_guard<std::mutex> guard(_mutex);
    std::copy(srcIn, srcIn + size, dest);
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::MinMax(T* minOut, T* maxOut)
{
    // must have = and < operator defined
    auto e = GetElements();
    if (e == 1) return Get(0);
    assert(1 < e);

    size_t start;
    T s, l;
    s = l = Get(0);
    T mi, ma;
    // initial check so we can index by 2 in loop
    if (e % 2 != 0)
    {
        // odd
        mi = ma = Get(0);
        start = 1;
    }
    else
    {
        // even
        const T& e2 = Get(1);
        mi = s < e2 ? s : e2;
        ma = e2 < l ? l : e2;
        start = 2;
    }

    --e;
    for (size_t i = start; i < e; i += 2)
    {
        const T& e1 = Get(i);
        const T& e2 = Get(i + 1);

        if (e1 < e2)
        {
            s = e1; l = e2;
        }
        else
        {
            s = e2; l = e1;
        }
        if (mi > s) mi = s;
        if (ma < l) ma = l;
    }

    *minOut = mi;
    *maxOut = ma;
    return;
}

template<typename T, std::size_t align>
inline T MemoryBlock<T, align>::Min()
{
    // must have = and < operator defined
    auto e = GetElements();
    if (e == 1) return Get(0);
    assert(e > 1);

    size_t start;
    T s = Get(0);
    T mi;
    // initial check so we can index by 2 in loop
    if (e % 2 != 0)
    {
        // odd
        mi = s;
        start = 1;
    }
    else
    {
        // even
        const T& e2 = Get(1);
        mi = s < e2 ? s : e2;
        start = 2;
    }

    --e;
    for (size_t i = start; i < e; i += 2)
    {
        const T& e1 = Get(i);
        const T& e2 = Get(i + 1);
        s = e1 < e2 ? e1 : e2;
        if (mi > s) mi = s;
    }
    return mi;
}

template<typename T, std::size_t align>
inline T MemoryBlock<T, align>::Max()
{
    // must have = and > operator defined
    auto e = GetElements();
    if (e == 1) return Get(0);
    assert(1 < e);

    size_t start;
    T l = Get(0);
    T ma;
    // initial check so we can index by 2 in loop
    if (e % 2 != 0)
    {
        // odd
        ma = l;
        start = 1;
    }
    else
    {
        // even
        const T& e2 = Get(1);
        ma = e2 < l ? l : e2;
        start = 2;
    }

    --e;
    for (size_t i = start; i < e; i += 2)
    {
        const T& e1 = Get(i);
        const T& e2 = Get(i + 1);
        l = e2 < e1 ? e1 : e2;
        if (ma < l) ma = l;
    }
    return ma;
}
// Memory helpers
template<typename T, std::size_t align>
inline uint8_t MemoryBlock<T, align>::Read1Byte(std::size_t elem)
{
    int value = Get(elem);
    return (uint8_t)value;
}

template<typename T, std::size_t align>
inline uint16_t MemoryBlock<T, align>::Read2ByteWord(std::size_t elem)
{
    auto addr = &Get(elem);
    return *(reinterpret_cast<uint16_t*>(addr));
}

template<typename T, std::size_t align>
inline uint32_t MemoryBlock<T, align>::Read4ByteDword(std::size_t elem)
{
    auto addr = &Get(elem);
    return *(reinterpret_cast<uint32_t*>(addr));
}

template<typename T, std::size_t align>
inline float MemoryBlock<T, align>::ReadFloat(size_t elem)
{
    auto addr = reinterpret_cast<float*>(&Get(elem));
    return *(addr);
}

template<typename T, std::size_t align>
inline uint64_t MemoryBlock<T, align>::Read8ByteDDword(std::size_t elem)
{
    auto addr = &Get(elem);
    return *(reinterpret_cast<uint64_t*>(addr));
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Write1Byte(size_t elem, uint8_t byteIn)
{
    auto addr = reinterpret_cast<uint8_t*>(&Get(elem));
    *addr = byteIn;
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Write2ByteWord(size_t elem, uint16_t wordIn)
{
    auto addr = reinterpret_cast<uint16_t*>(&Get(elem));
    *addr = wordIn;
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Write4ByteDword(size_t elem, uint32_t dwordIn)
{
    auto addr = reinterpret_cast<uint32_t*>(&Get(elem));
    *addr = dwordIn;
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Write8ByteDword(size_t elem, uint64_t ddwordIn)
{
    auto addr = reinterpret_cast<uint64_t*>(&Get(elem));
    *addr = ddwordIn;
}

// File helpers
template<typename T, std::size_t align>
inline uint8_t MemoryBlock<T, align>::Read1Byte(std::ifstream& dataFileIn)
{
    int value = dataFileIn.get();
    if (value != EOF)
        return (uint8_t)value;
    // mask 0xff
    else return 0;
}

template<typename T, std::size_t align>
inline uint16_t MemoryBlock<T, align>::Read2ByteWord(std::ifstream& dataFileIn)
{
    // little endian ordering
    auto b1 = Read1Byte(dataFileIn);
    auto b2 = Read1Byte(dataFileIn);
    if (dataFileIn.fail()) return 0;
    // mask 0xffff
    return ((b2 << 8) | b1);
}

template<typename T, std::size_t align>
inline uint32_t MemoryBlock<T, align>::Read4ByteDword(std::ifstream& dataFileIn)
{
    // little endian ordering
    auto b1 = Read1Byte(dataFileIn);
    auto b2 = Read1Byte(dataFileIn);
    auto b3 = Read1Byte(dataFileIn);
    auto b4 = Read1Byte(dataFileIn);
    if (dataFileIn.fail()) return 0;
    // mask 0xffffffff
    return ((b4 << 24) | (b3 << 16) | (b2 << 8) | b1);
}

template<typename T, std::size_t align>
inline float MemoryBlock<T, align>::ReadFloat(std::ifstream& dataFileIn)
{
    float f;
    dataFileIn.read(reinterpret_cast<char*>(&f), 4);
    return f;
}

template<typename T, std::size_t align>
inline uint64_t MemoryBlock<T, align>::Read8ByteDDword(std::ifstream& dataFileIn)
{
    // little endian ordering
    auto b1 = Read1Byte(dataFileIn);
    auto b2 = Read1Byte(dataFileIn);
    auto b3 = Read1Byte(dataFileIn);
    auto b4 = Read1Byte(dataFileIn);
    auto b5 = Read1Byte(dataFileIn);
    auto b6 = Read1Byte(dataFileIn);
    auto b7 = Read1Byte(dataFileIn);
    auto b8 = Read1Byte(dataFileIn);
    if (dataFileIn.fail()) return 0;
    // mask 0xffffffffffffffff
    return ((b8 << 56) | (b7 << 48) | (b6 << 40) | (b5 << 32) | (b4 << 24) | (b3 << 16) | (b2 << 8) | b1);
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Write1Byte(std::ofstream& dataFileOut, uint8_t byteIn)
{
    dataFileOut.put(byteIn);
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Write2ByteWord(std::ofstream& dataFileOut, uint16_t wordIn)
{
    // little endian ordering
    Write1Byte(dataFileOut, wordIn & 0x00FF);
    Write1Byte(dataFileOut, (wordIn & 0xFF00) >> 8);
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Write4ByteDword(std::ofstream& dataFileOut, uint32_t dwordIn)
{
    // little endian ordering
    Write1Byte(dataFileOut, dwordIn & 0xFF);
    Write1Byte(dataFileOut, (dwordIn >> 8) & 0xFF);
    Write1Byte(dataFileOut, (dwordIn >> 16) & 0xFF);
    Write1Byte(dataFileOut, (dwordIn >> 24) & 0xFF);
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Write8ByteDDword(std::ofstream& dataFileOut, uint64_t ddwordIn)
{
    // little endian ordering
    Write1Byte(dataFileOut, ddwordIn & 0xFF);
    Write1Byte(dataFileOut, (ddwordIn >> 8) & 0xFF);
    Write1Byte(dataFileOut, (ddwordIn >> 16) & 0xFF);
    Write1Byte(dataFileOut, (ddwordIn >> 24) & 0xFF);
    Write1Byte(dataFileOut, (ddwordIn >> 32) & 0xFF);
    Write1Byte(dataFileOut, (ddwordIn >> 40) & 0xFF);
    Write1Byte(dataFileOut, (ddwordIn >> 48) & 0xFF);
    Write1Byte(dataFileOut, (ddwordIn >> 56) & 0xFF);
}
}