#pragma once

#include "KArcCacheNode.h"
#include <unordered_map>
#include <map>
#include <mutex>
#include <list>
#include <cstddef>

namespace KamaCache
{

template<typename Key, typename Value>
class ArcLfuPart
{
public:
    using NodeType = ArcNode<Key, Value>;
    using NodePtr = std::shared_ptr<NodeType>;
    using NodeMap = std::unordered_map<Key, NodePtr>;
    using FreqMap = std::map<size_t, std::list<NodePtr> >;

private:
    std::size_t capacity_;
    std::size_t ghostCapacity_;
    std::size_t transformThreshold_;
    std::size_t minFreq_;
    std::mutex mutex_;

    NodeMap mainCache_;
    NodeMap ghostCache_;
    FreqMap freqMap_;

    NodePtr ghostHead_;
    NodePtr ghostTail_;

void initializeLists()
{
    ghostHead_ = std::make_shared<NodeType> ();
    ghostTail_ = std::make_shared<NodeType> ();
    ghostHead_->next_ = ghostTail_;
    ghostTail_->prev_ = ghostHead_; 
}
void updateNodeFrequency(NodePtr node)
{
    std::size_t oldFreq = node->getAccesscount();
    node->incrementAccessCount();
    std::size_t newFreq = node->getAccesscount();

    auto & oldlist = freqMap_[oldFreq];
    oldlist.remove(node);
    if ( oldlist.empty() )
    {
        freqMap_.erase(oldFreq);
        if ( oldFreq == minFreq_)
        {
            minFreq_ = newFreq;
        }
    }

    if ( freqMap_.find( newFreq) == freqMap_.end() )
    {
        freqMap_[newFreq] = std::list<NodePtr>();
    }

    freqMap_[newFreq].push_back(node);
}


bool updateExistingNode(NodePtr node, const Value & value)
{
    node->setValue(value);
    updateNodeFrequency(node);
    return true;
}

void removeFromGhost(NodePtr node)
{
    node->prev_->next_ = node->next_;
    node->next_->prev_ = node->prev_;
}

void removeOldestGhost()
{
    NodePtr oldestGhost = ghostHead_->next_;
    if (oldestGhost != ghostTail_)
    {
        removeFromGhost(oldestGhost);
        ghostCache_.erase(oldestGhost->getKey());
    }
}

void addToGhost(NodePtr node)
{
    node->next_ = ghostTail_;
    node->prev_ = ghostTail_->prev_;
    ghostTail_->prev_->next_ = node;
    ghostTail_->prev_ = node;
    ghostCache_[node->getKey()] = node;
}

void evictLeastFrequent()
{
    if ( freqMap_.empty() )
    {
        return;
    }
    auto & minFreqList = freqMap_[minFreq_];
    if ( minFreqList.empty() )
    {
        return;
    }

    NodePtr leastNode = minFreqList.front();
    minFreqList.pop_front();

    if ( minFreqList.empty())
    {
        freqMap_.erase(minFreq_);
        if ( !freqMap_.empty() )
        {
            minFreq_ = freqMap_.begin()->first;
        }
    }

    if (ghostCache_.size() >= capacity_)
    {
        removeOldestGhost();
    }
    addToGhost(leastNode);

    mainCache_.erase(leastNode->getKey());

}

bool addNewNode(const Key & key, const Value & value)
{
    if (mainCache_.size() >= capacity_)
    {
        evictLeastFrequent();
    }

    NodePtr newNode = std::make_shared<NodeType> (key, value);
    mainCache_[key] = newNode;

    if ( freqMap_.find( 1) == freqMap_.end() )
    {
        freqMap_[1] = std::list<NodePtr>();
    }

    freqMap_[1].push_back(newNode);
    minFreq_ = 1;

    return true;
}

public:
    explicit ArcLfuPart(std::size_t capacity, std::size_t transformThreshold)
    : capacity_(capacity),
    ghostCapacity_(capacity_),
    transformThreshold_(transformThreshold),
    minFreq_(0)
    {
        initializeLists();
    }

    bool put(Key key, Value value)
    {
        if ( capacity_ <= 0)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        auto it = mainCache_.find(key);
        if ( it != mainCache_.end() )
        {
            updateExistingNode(it->second, value);
        }

        return addNewNode(key, value);
    }

    bool get(Key key, Value & value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = mainCache_.find(key);
        if ( it != mainCache_.end() )
        {
            value = it->second->getValue();
            return true;
        }

        return false;
    }

    bool checkGhost(Key key)
    {
        auto it = ghostCache_.find(key);

        if (it != ghostCache_.end() )
        {
            removeFromGhost(it->second);
            ghostCache_.erase(it->second->getKey());
            return true;
        } 

        return false;
    }

    void increaseCapacity(){ ++capacity_; }

    bool decreaseCapacity()
    {
        if (capacity_ <= 0) return false;
        if (mainCache_.size() == capacity_)
        {
            evictLeastFrequent();
        }
        --capacity_;
        return true;
    }
}; 
}