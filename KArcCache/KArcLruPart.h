#pragma once

#include "KArcCacheNode.h"
#include <unordered_map>
#include <mutex>
#include <cstddef>

namespace KamaCache
{

template<typename Key, typename Value>
class ArcLruPart
{
public:
    using NodeType = ArcNode<Key, Value>;
    using NodePtr = std::shared_ptr<NodeType>;
    using NodeMap = std::unordered_map<Key, NodePtr>;
private:
    std::size_t capacity_;
    std::size_t ghostCapacity_;
    std::size_t transformThreshold_;
    std::mutex mutex_;

    NodeMap mainCache_;
    NodeMap ghostCache_;

    NodePtr mainHead_;
    NodePtr mainTail_;

    NodePtr ghostHead_;
    NodePtr ghostTail_;
    void initializeLists()
    {
        mainHead_ = std::make_shared<NodeType>();
        mainTail_ = std::make_shared<NodeType>();
        mainHead_->next_ = mainTail_;
        mainTail_->prev_ = mainHead_;

        ghostHead_ = std::make_shared<NodeType>();
        ghostTail_ = std::make_shared<NodeType>();
        ghostHead_->next_ = mainTail_;
        mainTail_->prev_ = mainHead_;
    }

    void addToFront(NodePtr node)
    {
        node->next_ = mainHead_->next_;
        node->prev_ = mainHead_;
        mainHead_->next_->prev_ = node;
        mainHead_->next_ = node;
    }

    void moveToFront(NodePtr node)
    {
        node->prev_->next_ = node->next_;
        node->next_->prev_ = node->prev_;

        addToFront(node);
    }
    
    bool updateExistingNode(NodePtr node, const Value & value)
    {
        node->setValue(value);
        moveToFront(node);
        return true;
    }

    void removeFromMain(NodePtr node)
    {
        node->prev_->next_ = node->next_;
        node->next_->prev_ = node->prev_;
    }

    void removeFromGhost(NodePtr node)
    {
        node->prev_->next_ = node->next_;
        node->next_->prev_ = node->prev_;
    }

    void removeOldestGhost()
    {
        NodePtr oldestGhost = ghostTail_->prev_;
        if ( oldestGhost == ghostHead_ )
        {
            return;
        }
        removeFromGhost(oldestGhost);
        ghostCache_.erase(oldestGhost->getKey());
    }

    void addToGhost(NodePtr node)
    {
        node->accesscount_ = 1;
        node->next_ = ghostHead_->next_;
        node->prev_ = ghostHead_;
        ghostHead_->next_->prev_ = node;
        ghostHead_->next_ = node;
        ghostCache_[node->getKey()] = node;
    }

    void evictLeastRecent()
    {
        NodePtr leastRecent = mainTail_->prev_;
        if ( leastRecent == mainHead_ )
        {
            return;
        }

        removeFromMain(leastRecent);

        if (ghostCache_.size() >= ghostCapacity_)
        {
            removeOldestGhost();
        }
        addToGhost(leastRecent);
        mainCache_.erase(leastRecent->getKey());
    }

    bool addNewNode(const Key & key, const Value & value)
    {
        if ( mainCache_.size() == capacity_)
        {
            evictLeastRecent();
        }

        NodePtr newNode = std::make_shared<NodeType> (key, value);
        mainCache_[key] = newNode;
        addToFront(newNode);
        return true;
    }

    bool updateNodeAccess(NodePtr node)
    {
        moveToFront(node);
        node->incrementAccessCount();
        return node->getAccesscount() >= transformThreshold_;
    }


public:
    explicit ArcLruPart(std::size_t capacity, std::size_t transformThreshold)
    : capacity_(capacity),
    ghostCapacity_(capacity),
    transformThreshold_(transformThreshold)
    {
        initializeLists();
    }

    bool put(Key key, Value value)
    {
        if (capacity_ <= 0)
        {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = mainCache_.find(key);
        if ( it != mainCache_.end() )
        {
            return updateExistingNode(it->second, value);
        }

        return addNewNode(key, value);
    }

    bool get(Key key, Value & value, bool & shouldTransform)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = mainCache_.find(key);
        if ( it != mainCache_.end() )
        {
            shouldTransform = updateNodeAccess(it->second);
            value = it->second->getValue();
            return true;
        }

        return false;
    }

    bool checkGhost(Key key)
    {
        auto it = ghostCache_.find(key);
        if ( it != ghostCache_.end() )
        {
            removeFromGhost(it->second);
            ghostCache_.erase(key);
            return true;
        }

        return false;
    }

    void increaseCapacity(){ ++capacity_;}

    bool decreaseCapacity()
    {
        if (capacity_ <= 0) return false;
        if ( mainCache_.size() == capacity_ )
        {
            evictLeastRecent();
        }
        --capacity_;
        return true;
    }

    



};
}