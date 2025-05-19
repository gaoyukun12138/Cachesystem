#pragma once

#include <memory>
#include <cstddef>

namespace KamaCache
{


template<typename Key, typename Value>
class ArcNode
{
private:
    Key key_;
    Value value_;
    std::size_t accesscount_;
    std::shared_ptr<ArcNode> prev_;
    std::shared_ptr<ArcNode> next_;

public:
    ArcNode()
    : accesscount_(1), prev_(nullptr), next_(nullptr)
    {}

    ArcNode(Key key, Value value)
    : key_(key), value_(value), accesscount_(1), prev_(nullptr), next_(nullptr)
    {}

    Key getKey() const { return key_;}
    Value getValue() const { return value_; }
    std::size_t getAccesscount() const { return accesscount_; }


    void setValue(const Value value) { value_ = value; }
    void incrementAccessCount() { ++accesscount_; }
    
    template<typename K, typename V> friend class ArcLruPart;
    template<typename K, typename V> friend class ArcLfuPart;
};    
}