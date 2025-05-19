#pragma once

#include <cstring>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <thread>
#include <cmath>
#include "KICachePolicy.h"


namespace KamaCache
{

// 前向声明
template<typename Key, typename Value> class KLruCache;

template <typename Key, typename Value>
class LruNode
{
private:
    
    Key key_;
    Value value_;
    std::size_t accesscount_;
    std::shared_ptr<LruNode<Key, Value> > prev_;
    std::shared_ptr<LruNode<Key, Value> > next_;
public:
    LruNode(Key key, Value value) : key_(key), value_(value), accesscount_(1)
    {
        prev_ = nullptr;
        next_ = nullptr;
    }
    Key getKey() const {return key_;}
    Value getValue() const {return value_; }
    void setValue(const Value & value) {value_ = value; }
    std::size_t getAccessCount() const { return accesscount_; }
    void incrementAccessCount() { ++accesscount_; }

    friend class KLruCache<Key, Value>;
};

template<typename Key, typename Value>
class KLruCache : public KICachePolicy<Key, Value>
{
public:
    using LruNodeType = LruNode<Key, Value>;
    using NodePtr = std::shared_ptr<LruNodeType>;
    using NodeMap = std::unordered_map<Key, NodePtr>;
private:
    int  capacity_;
    NodeMap nodemap_;
    std::mutex mutex_;
    NodePtr dummyhead_;
    NodePtr dummytail_;

    void initializeList()
    {
        dummyhead_ = std::make_shared<LruNodeType>(Key(), Value());
        dummytail_ = std::make_shared<LruNodeType>(Key(), Value());
        dummyhead_->next_ = dummytail_;
        dummytail_->prev_ = dummyhead_;
    }

    void updateExistingNode(NodePtr node, const Value & value)
    {
        node->setValue(value);
        moveToMostRecent(node);
    }

    void addNewNode(const Key & key, const Value & value)
    {
        if ( nodemap_.size() == capacity_)
        {
            evictLeastRecent();
        }

        NodePtr newnode = std::make_shared<LruNodeType> (key, value);
        insertNode(newnode);
        nodemap_[key] = newnode;
    }

    void moveToMostRecent( NodePtr node )
    {
        removeNode(node);
        insertNode(node);
    }

    void removeNode(NodePtr node)
    {
        node->prev_->next_ = node->next_;
        node->next_->prev_ = node->prev_;
    }

    void insertNode(NodePtr node)
    {
        node->next_ = dummytail_;
        node->prev_ = dummytail_->prev_;
        dummytail_->prev_->next_ = node;
        dummytail_->prev_ = node;
    }

    void evictLeastRecent()
    {
        NodePtr leastRecent = dummyhead_->next_;
        removeNode(leastRecent);
        nodemap_.erase(leastRecent->getKey());
    }

    
public:
    KLruCache(int capacity) : capacity_(capacity)
    {
        initializeList();
    }
    ~KLruCache() override = default;

    void put(Key key, Value value) override
    {
        if ( capacity_ <= 0 )
        {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        auto it = nodemap_.find(key);
        if ( it != nodemap_.end() )
        {
            updateExistingNode(it->second, value);
            return;
        }

        addNewNode(key, value);
    }

    bool get(Key key, Value & value) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = nodemap_.find(key);
        if ( it != nodemap_.end() )
        {
            moveToMostRecent(it->second);
            value = it->second->getValue();
            return true;
        }

        return false;
    }

    Value get(Key key) override
    {
        Value value{};
        get(key, value);
        return value;
    }

    void remove(Key key)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = nodemap_.find(key);
        if (it != nodemap_.end() )
        {
            removeNode(it->second);
            nodemap_.erase(it->second->getKey());
        }
    }
};


template<typename Key, typename Value>
class KLruKCache : public KLruCache<Key, Value>
{
private:
    int k_;
    std::unique_ptr<KLruCache<Key, std::size_t> > historylist_;

public:
    KLruKCache(int capacity, int historyCapacity, int k)
    : KLruCache<Key, Value> (capacity),
    historylist_(std::make_unique<KLruCache<Key, std::size_t> > (historyCapacity) ),
    k_(k)
    {}

    Value get(Key key)
    {
        int historycount = historylist_->get(key);
        
        historylist_.put(key, ++historycount);

        return KLruCache<Key, Value>::get(key);
    }

    void put(Key key, Value value)
    {
        if (KLruKCache<Key, Value>::get(key) != "")
        {
            return KLruCache<Key, Value>::put(key, value);
        }

        int historycount = historylist_->get(key);
        historylist_.put(key, ++historycount);

        if ( historycount >= k_)
        {
            historylist_->remove(key);
            KLruKCache<Key, Value>::put(key, value);
        }
    }

};

template<typename Key, typename Value>
class KHashLruCaches
{
private:
    size_t capacity_;
    int sliceNum_;
    std::vector<std::unique_ptr<KLruCache<Key, Value> > > lruSliceCaches_;

    size_t Hash(Key key)
    {
        std::hash<Key> hashfunc;
        return hashfunc(key);
    }

public:
    KHashLruCaches(size_t capacity, int sliceNum)
    : capacity_(capacity), 
    sliceNum_(sliceNum > 0 ? sliceNum : std::thread::hardware_concurrency())
    {
        size_t silceSize = std::ceil(capacity / static_cast<double> (sliceNum_));
        for ( int i = 0; i < sliceNum_; i++)
        {
            lruSliceCaches_.emplace_back(new KLruCache<Key, Value> (silceSize));
        }
    }

    void put(Key key, Value value)
    {
        std::size_t sliceindex = Hash(key) % sliceNum_;
        lruSliceCaches_[sliceindex]->put(key, value);
    }

    bool get(Key key, Value & value)
    {
        std::size_t sliceindex = Hash(key) % sliceNum_;
        return lruSliceCaches_[sliceindex]->get(key, value);
    }

    Value get(Key key)
    {
        Value value{};
        get(key, value);
        return value;
    }
};

}




