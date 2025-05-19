#pragma once


#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#include <climits>

#include "KICachePolicy.h"

namespace KamaCache
{
template<typename Key, typename Value> class KLfuCache;

template<typename Key, typename Value>
class FreqList
{
public:
    struct Node
    {
        Key key_;
        Value value_;
        int freq_;
        std::shared_ptr<Node> pre_;
        std::shared_ptr<Node> next_;

        Node() : freq_(1), pre_(nullptr), next_(nullptr)
        {
        }

        Node(Key key, Value value) : key_(key), value_(value), freq_(1)
        {
            pre_ = nullptr;
            next_ = nullptr;
        }
    };
private:

    int List_freq_;
    std::shared_ptr<Node> head_;
    std::shared_ptr<Node> tail_;
public:
    explicit FreqList(int n) : List_freq_(n)
    {
        head_ = std::make_shared<Node> ();
        tail_ = std::make_shared<Node> ();
        head_->next_ = tail_;
        tail_->pre_ = head_;
    }

    bool isEmpty() const{
        return head_->next_ == tail_;
    }

    void addNode( std::shared_ptr<Node> node )
    {
        if ( !node || !head_ || !tail_ )
        {
            return;
        }

        std::shared_ptr<Node> next = head_->next_;
        node->pre_ = head_;
        node->next_ = next;
        head_->next_ = node;
        next->pre_ = node; 
    }

    void removeNode (std::shared_ptr<Node> node)
    {
        if ( !node || !head_ || !tail_)
        {
            return;
        }
        if ( !node->pre_ || !node->next_)
        {
            return;
        }

        node->pre_->next_ = node->next_;
        node->next_->pre_ = node->pre_;
    }

    std::shared_ptr<Node> getFirstNode() const
    {
        return tail_->pre_;
    }

    friend class KLfuCache<Key, Value>;
};

template<typename Key, typename Value>
class KLfuCache : public KICachePolicy<Key, Value>
{
public:
    using Node = typename FreqList<Key, Value>::Node; // 
    using NodePtr = std::shared_ptr<Node>;
    using Nodemap = std::unordered_map<Key, NodePtr>;
private:
    int capacity_;
    int minFreq_;
    int maxAverageNum_;
    int curAverageNum_;
    int curTotalNum_;
    std::mutex mutex_;
    Nodemap nodeMap_;
    std::unordered_map<int, FreqList<Key, Value>* > freqToFreqList_;

    void decreaseFreqNum(int num)
    {
        curTotalNum_ -= num;
        if ( nodeMap_.empty( ))
        {
            curAverageNum_ = 0;
        }
        else
        {
            curAverageNum_ = curTotalNum_ / nodeMap_.size();
        }
    }

    void kickOut()
    {
        NodePtr node = freqToFreqList_[minFreq_]->getFirstNode();
        removeFromFreqList(node);
        nodeMap_.erase(node->key_);
        decreaseFreqNum(node->freq_);
    }

    void removeFromFreqList(NodePtr node)
    {
        if ( !node )
        {
            return;
        }
        auto freq = node->freq_;
        freqToFreqList_[freq]->removeNode(node);
    }

    void addToFreqList(NodePtr node)
    {
        if ( !node )
        {
            return;
        }
        auto freq = node->freq_;

        if ( freqToFreqList_.find(freq) == freqToFreqList_.end() )
        {
            freqToFreqList_[freq] = new FreqList<Key, Value> (freq);
        }

        freqToFreqList_[freq]->addNode(node);
    }

    void updateMinFreq()
    {
        minFreq_ = INT8_MAX;
        for ( const auto & pair : freqToFreqList_ )
        {
            if ( pair.second && !pair.second->isEmpty() )
            {
                minFreq_ = std::min(minFreq_, pair.first);
            }
        }

        if ( minFreq_ == INT8_MAX)
        {
            minFreq_ = 1;
        }

    }

    void handleOverMaxAverageNum()
    {
        if ( nodeMap_.empty() )
        {
            return;
        }

        for ( auto it = nodeMap_.begin(); it != nodeMap_.end(); it++)
        {
            if ( !it->second)
            {
                continue;
            }

            NodePtr node = it->second;
            removeFromFreqList(node);
            node->freq_ -= maxAverageNum_ / 2;
            if ( node->freq_ < 1) node->freq_ = 1;

            addToFreqList(node);
        }

        updateMinFreq();
    }

    void addFreqNum()
    {
        curTotalNum_++;
        if (nodeMap_.empty())
        {
            curAverageNum_ = 0;
        }
        else
        {
            curAverageNum_ = curTotalNum_ / nodeMap_.size();
        }

        if ( curAverageNum_ > maxAverageNum_)
        {
            handleOverMaxAverageNum();
        }

    }

    void getInternal(NodePtr node, Value & value)
    {
        value = node->value_;
        removeFromFreqList(node);
        node->freq_++;
        addToFreqList(node);

        if ( node->freq_ - 1 == minFreq_ && freqToFreqList_[node->freq_-1]->isEmpty() )
        {
            minFreq_++;
        }

        addFreqNum();
    }

    void putInternal(Key key, Value value)
    {
        if ( nodeMap_.size() == capacity_)
        {
            kickOut();
        }

        NodePtr node = std::make_shared<Node> (key, value);
        nodeMap_[key] = node;
        addToFreqList(node);
        addFreqNum();
        minFreq_ = std::min(minFreq_, 1);
    }

public:
    KLfuCache(int capacity, int maxAverageNum = 5)
    : capacity_(capacity), minFreq_(INT8_MAX), maxAverageNum_(maxAverageNum),
    curTotalNum_(0), curAverageNum_(0)
    {
    }

    ~KLfuCache() override = default;

    void put(Key key, Value value) override
    {
        if (capacity_ == 0)
        {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = nodeMap_.find(key);
        if ( it != nodeMap_.end() )
        {
            it->second->value_ = value;
            getInternal(it->second, value);
            return;
        }

        putInternal(key, value);
    }

    bool get(Key key, Value & value) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = nodeMap_.find(key);
        if ( it != nodeMap_.end() )
        {
            getInternal(it->second, value);
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

    void purge()
    {
        nodeMap_.clear();
        freqToFreqList_.clear();
    }
};

template<typename Key, typename Value>
class KHashLfuCache
{
private:
    std::size_t Hash(Key key)
    {
        std::hash<Key> hashFunc;
        return hashFunc(key);
    }

    size_t capacity_;
    int sliceNum_;
    std::vector< std::unique_ptr<KLfuCache<Key, Value> > > lfuSliceCaches;

public:
    KHashLfuCache(size_t capacity, int sliceNum, int maxAvergeNum = 10)
    : capacity_(capacity), 
    sliceNum_( sliceNum_ > 0 ? sliceNum : std::thread::hardware_concurrency())
    {
        std::size_t sliceSize = std::ceil(capacity_/static_cast<double> (sliceNum_));
        for ( int i = 0; i < sliceSize; i++)
        {
            lfuSliceCaches.emplace_back(KLfuCache<Key, Value> ( sliceSize, maxAvergeNum) );

        }
    }

    void put(Key key, Value value)
    {
        std::size_t sliceIndex = Hash(key) % sliceNum_;
        lfuSliceCaches[sliceIndex]->put(key, value);
    }

    bool get(Key key, Value & value)
    {
        size_t sliceIndex = Hash(key) % sliceNum_;
        return lfuSliceCaches[sliceIndex]->get(key, value);

    }

    Value get(Key key)
    {
        Value value{};
        get(key, value);
        return value;
    }

    void purge()
    {
        for ( auto & lfuSliceCache : lfuSliceCaches)
        {
            lfuSliceCache->purge();
        }
    }
};

}