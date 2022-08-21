/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          MemoryBlock

Description:    Dynamic (eg. has an over allocated reserve) CPU memory buffer 
                that can be aligned and has proper move and copy assignment semantics.  

                This class differs from std::vector<T> in that it supports strides
                that we will be using to work with vertex buffers of dynamic
                structures.  Down side is you don't get the standard's algorithims
                to sort and find info in the vector.  This class wraps a memory 
                pointer and tracks _length in use and _capacity available.

                MemoryBlock<T> has operator overloading of + and += to Append

Remarks:        Also supports read and write to files in native T type and
                raw binary as well as writing in text.  You may find it convienent to define 
                data into MemoryBlock<T> and then merge that into MemoryBlock<uint8_t>
                when finished. An advantage of this is stride use for 
                reinterpreting the data for vertex buffers and to have one common file 
                read and write code for every buffer. Extension into our memory pool class 
                for the engine later on is simple since it builds upon this class and allocations
                to a fixed pool manager is done by just swapping out the class type of MemoryBlock 
                for the pool class type.

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
                7/30/2022 Implemented json support for file saving and loading and network transport

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

namespace King 
{
template<typename T, std::size_t align = 16> // bytes
class alignas(align) MemoryBlock
{
    /* variables */
private:
    size_t              _length = 0; // number of T elements assigned within data allocation
    size_t              _capacity = 0; //number of T elements reserved for in data allocation
    size_t              _stride = 1; // number of T elements to skip per operator [] indexing (convenient). Note Size() is now depreciated to make this clearer in loops
    T*                  _data = nullptr; // pointer is a multiple of alignment and only alligned if T new operator is also aligned
    mutable std::mutex  _mutex;
    /* methods */
public:
    // construction/life cycle
    explicit MemoryBlock() : _capacity(0) { Allocate(2); }
    MemoryBlock(nullptr_t) : MemoryBlock() {} // allow nullptr construct since that is the default
    explicit MemoryBlock(size_t elementCount, size_t byteStridePerElement = 1) : _stride(byteStridePerElement) { Allocate(elementCount * _stride); Fill(); }
    explicit MemoryBlock(const MemoryBlock& other) { *this = other; } // Copy constructor
    explicit MemoryBlock(MemoryBlock&& other) noexcept { *this = std::move(other); } // Move constructor
    MemoryBlock(std::initializer_list<T> il) : _length(il.size()) { Allocate(_length); std::copy(std::begin(il), std::end(il), _data); }
    
    ~MemoryBlock() { Destroy(); }

    // operators
    void * operator new (size_t size) { return _aligned_malloc(size, align); }
    void   operator delete (void *p) { _aligned_free(static_cast<MemoryBlock*>(p)); }
    
    const T& operator [](size_t i) const { assert(i*_stride<_length); return _data[i*_stride]; } // accessor 
    T& operator [](size_t i) { assert(i*_stride<_capacity); return _data[i*_stride]; } // assignment
    
    T* operator->() { return _data; } // data pointer access
    const T* operator->() const { return _data; } // const data pointer access
    explicit operator bool() const { return (bool)_length; } // valid
    bool operator !() const { return !(bool)_data || !(bool)_length; } // invalid
    inline MemoryBlock& operator+=(const MemoryBlock& other);
    inline MemoryBlock operator+(const MemoryBlock& other);

    inline MemoryBlock& operator=(const T* other); // copy in our length worth of data from the pointer passed of type
    inline MemoryBlock& operator=(const MemoryBlock& other);
    inline MemoryBlock& operator=(MemoryBlock&& other) noexcept;

    // Functionality
    inline void                 Initialize(size_t lengthIn) { Allocate(lengthIn); Fill(); }
    inline void                 Reserve(size_t capacityIn) { ReAllocate(capacityIn); }
    inline void                 Destroy();
    
    // dynamic allocations
    inline void                 PushBack(const T& valueIn);
    inline void                 PushBack(const T&& valueIn);
    template<typename... Args> inline T& EmplaceBack(Args&&... args);
    inline void                 PopBack();

    // block methods
    inline void                 Clear(); // calls destructor of each object
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

    inline bool                 Read(std::ifstream& dataFileIn); // pass stream so we can pack multiple MemoryBlocks into one file
    inline bool                 Write(std::ofstream& outfileIn);

    inline bool                 ReadRawBinary(std::string& fileNameIn); // length unknown, so read entire file and use file length as byte size
    inline bool                 WriteRawBinary(std::string fileNameIn);

    inline bool                 WriteText(std::string fileNameIn); // for easy debugging of data

    [[deprecated("Use GetLength() or GetElements() depending on use instead.")]]
    auto Size() { return _length; } // note, use GetElements() for stride lengths
    // Accessors
    T&                          Get(size_t i) { assert(i * _stride < _length); return _data[i * _stride]; }
    const T&                    Get(size_t i) const { assert(i * _stride < _length); return _data[i * _stride]; }
    T&                          GetData() { return *_data; }
    const T&                    GetData() const { return *_data; }
    const auto&                 GetLength() const { return _length; } // number of T data
    const auto&                 GetCapacity() const { return _capacity; } // number of T data
    const auto&                 GetStride() const { return _stride; } // number of T elements to skip with operator[] indexing
    const auto                  GetByteSize() const { return _length * sizeof(T); }
    const auto                  GetElements() const { return _length / _stride; } // number of stride elements
    const auto&                 GetElement(size_t i) const { return operator [](i); }
    const auto&                 GetFront() const { assert(_length); return _data[0]; }
    const auto&                 GetBack() const { return operator [](GetElements() - 1); }
    // Assignments
    void                        SetLength(const size_t& lengthIn); // note: this does not construct in place objects, use this method only if you have copied the data into the length that grows
    void                        SetStride(const size_t& strideIn) { std::lock_guard<std::mutex> guard(_mutex); _stride = strideIn; }

private:
    void                        Allocate(const std::size_t capacityIn);
    void                        ReAllocate(const std::size_t capacityIn);
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
inline MemoryBlock<T, align>& MemoryBlock<T, align>::operator=(const T* other)
{
    assert(_length);
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
inline MemoryBlock<T, align>& MemoryBlock<T, align>::operator=(const MemoryBlock& other)
{
    // keeps our _capacity if > other._length
    if (this != &other)
    {
        if (_capacity < other._length)
        {
            Destroy();
            Allocate(other._length);
        }
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
    // note: this does not construct in place objects, use this method only if you have copied the data into the length that grows

    // grows?
    if (lengthIn > _capacity) 
        ReAllocate(lengthIn); 

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
    
    _length = lengthIn; 
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Allocate(const std::size_t capacityIn)
{
    Destroy();
    std::lock_guard<std::mutex> guard(_mutex);
    _capacity = capacityIn;
    // only aligned if T's new operator also aligns
    //try 
    //{
        _data = (T*) ::operator new(_capacity * sizeof(T));
    //}
    //catch (const std::bad_alloc& e) 
    //{
    //    std::cout << "MemoryBlock::Allocation failed: " << e.what() << '\n';
    //    assert(0); // temp, we really should handle this with a callback
    //}

    if (_capacity < _length)
        _length = _capacity;
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::ReAllocate(const std::size_t capacityIn)
{
    if (_capacity == capacityIn)
    {
        return;
    }

    // only aligned if T's new operator also aligns
    T* data = nullptr;
    //try
    //{
        data = (T*)::operator new(capacityIn * sizeof(T));
    //}
    //catch (const std::bad_alloc& e)
    //{
    //    std::cout << "MemoryBlock::ReAllocate failed: " << e.what() << '\n';
    //    assert(0); // temp, we really should handle this with a callback or just let the app fail
    //}

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
inline void MemoryBlock<T, align>::Destroy() 
{ 
    if (_capacity) 
    { 
        Clear();
        
        std::lock_guard<std::mutex> guard(_mutex);
        if (_data != nullptr)
            ::operator delete(_data, _capacity * sizeof(T));

        _data = nullptr; 
        _length = 0;
        _capacity = 0;
        //_stride = 1; leave this same as before Destroy so order of setting it does not matter. Often Detroy would be called implicitly
    } 
}

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
    if (_length >= _capacity)
        ReAllocate(_capacity + _capacity / 2);

    _data[_length] = std::move(valueIn);
    ++_length;
}

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
    if (_length >= _capacity)
        ReAllocate(_capacity + _capacity / 2);

    new(&_data[_length]) T(std::forward<Args>(args)...);
    return _data[_length++];
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Clear()
{
    std::lock_guard<std::mutex> guard(_mutex);
    while (_length > 0)
    {
        --_length;
        _data[_length].~T();
    }
    _length = 0;
}

template<typename T, std::size_t align>
inline void MemoryBlock<T, align>::Fill()
{
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

        std::lock_guard<std::mutex> guard(_mutex);
        //std::copy(other._data, other._data + other._length, _data + _length);
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

    //dataFileIn >> readLength >> readStride; // 6/27/2022 CHK error: not reading the values correctly, but the below works fine

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

}