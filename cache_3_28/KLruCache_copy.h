#pragma once

#include <memory>
#include <cstddef>
#include <mutex>
#include <unordered_map>
#include "KICachePolicy.h"



namespace Kamaspace
{

template<typename Key, typename Value> class LruCache; 

template<typename Key, typename Value> 
class Node
{
private:
    Key key_;
    Value value_;
    std::size_t accesscount_;
    std::shared_ptr<Node<Key, Value> > prev_;
    std::shared_ptr<Node<Key,Value> > next_;
public:
    Node(Key key, Value value) : key_(key), value_(value), accesscount_(1)
    {
        prev_ = nullptr;
        next_ = nullptr;
    }
    Key GetKey() {return key_;}
    Value GetValue() {return value_;}
    void SetValue(Value value) { value_ = value; }
    std::size_t GetAccessCount() { return accesscount_;}
    void AccessCountIncrease() { accesscount_++; }

    friend class LruCache<Key, Value>;
};

template <typename Key, typename Value>
class LruCache : public KamaCache::KICachePolicy<Key, Value>
{
public:
    using Node = Node<Key, Value>;
    using NodePtr = std::shared_ptr<Node>;
    using NodeMap = std::unordered_map<Key, NodePtr>;

private:
    std::size_t capacity_;
    std::mutex Mutex_;
    NodeMap NodeMap_;
    NodePtr dummyhead;
    NodePtr dummytail;

    void RemoveNode(NodePtr node)
    {
        node->next_->prev_ = node->prev_;
        node->prev_->next_ = node->next_;
         
    }

    void InsertNode(NodePtr node)
    {
        NodePtr next = dummyhead->next_;
        dummyhead->next_ = node;
        next->prev_ = node;
        node->next_ = next;
        node->prev_ = dummyhead;
    }

    void AddNewNode(Key key, Value value)
    {
        if ( NodeMap_.size() == capacity_)
        {
            NodePtr cur = dummytail->prev_;
            RemoveNode(cur);
            NodeMap_.erase(cur->key_);  
        }
        NodePtr newnode = std::make_shared<Node> (key, value);
        NodeMap_[key] = newnode;
        InsertNode(newnode);
    }

    void MoveToFront(NodePtr node)
    {
        RemoveNode(node);
        InsertNode(node);
    }

    void UpdateExistsNode(NodePtr node, Value value)
    {
        node->value_ = value;
        MoveToFront(node);

    }


public:
    LruCache(std::size_t capacity) : capacity_(capacity)
    {
        dummyhead = std::make_shared<Node> (-1, -1);
        dummytail = std::make_shared<Node> (-1, -1);
        dummyhead->next_ = dummytail;
        dummytail->prev_ = dummyhead;
    }

    ~LruCache() override = default;

    void put(Key key, Value value)
    {
        if ( capacity_ <= 0)
        {
            return;
        }
        std::lock_guard<std::mutex> lock(Mutex_);
        auto iter  = NodeMap_.find(key);
        if ( iter != NodeMap_.end() )
        {
            UpdateExistsNode(iter->second, value);
            return;
        }

        AddNewNode(key, value);
    }

    bool get(Key key, Value & value)
    {
        std::lock_guard<std::mutex> lock(Mutex_);
        auto iter = NodeMap_.find(key);
        if ( iter != NodeMap_.end() )
        {
            MoveToFront(iter->second);
            value = iter->second->GetValue();
            return true;
        }

        return false;
    }

    Value get(Key key)
    {
        Value value{};
        get(key, value);
        return value;
    }

    void remove(Key key)
    {
        std::lock_guard<std::mutex> lock(Mutex_);
        auto iter = NodeMap_.find(key);
        if ( iter != NodeMap_.end() )
        {
            RemoveNode( iter->second);
            NodeMap_.erase(iter->second->GetKey());
        }

    }


};

template<typename Key, typename Value>
class KlruKCache : public LruCache<Key, Value>
{
private:
    int k_;
    std::unique_ptr<LruCache<Key, std::size_t> > historycounts;

public:
    KlruKCache(std::size_t capacity, std::size_t maxhistory, int k)
    : k_(k),
    historycounts(std::make_unique<LruCache<Key, std::size_t> > (maxhistory)),
    LruCache<Key, Value> (capacity) 
    {
    }

    Value get(Key key)
    {
        int hiscount = historycounts->get(key);
        historycounts->put(key, hiscount++);


        return LruCache<Key, Value>::get(Key);
    }

    void put(Key key, Value value)
    {
        if ( LruCache<Key, Value> :: get(key) != "")
        {
            LruCache<Key, Value>::put(key, value);
        }

        int hiscount = historycounts->get(key);
        historycounts->put(key, hiscount++);
        if (hiscount > k_)
        {
            historycounts->remove(key);
            LruCache<Key, Value>::put(key, value);
        }
    }

};


}