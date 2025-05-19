#pragma once

#include "../cache_3_28/KICachePolicy.h"
#include "KArcLruPart.h"
#include "KArcLfuPart.h"
#include <memory>
#include <cstddef>

namespace KamaCache
{

template<typename Key, typename Value>
class KArcCache : public KICachePolicy<Key, Value>
{

 private:
    std::size_t capacity_;
    std::size_t transformThreshold_;
    std::unique_ptr<ArcLfuPart<Key, Value> > lfuPart;
    std::unique_ptr<ArcLruPart<Key, Value> > lruPart;

    bool checkGhostCaches(Key key)
    {
        bool inGhost = false;
        if ( lruPart->checkGhost(key))
        {
            if ( lfuPart->decreaseCapacity() )
            {
                lruPart->increaseCapacity();
            }
            inGhost = true;
        }
        else if ( lfuPart->checkGhost(key) )
        {
            if ( lruPart->decreaseCapacity() )
            {
                lfuPart->increaseCapacity();
            }
            inGhost = true;
        }

        return inGhost;
    }

public:
    explicit KArcCache(std::size_t capacity = 10, std::size_t transformThreshold = 2)
    : capacity_(capacity),
    transformThreshold_(transformThreshold),
    lruPart(std::make_unique<ArcLruPart<Key, Value> > (capacity, transformThreshold)),
    lfuPart(std::make_unique<ArcLfuPart<Key, Value> > (capacity, transformThreshold))
    {}

    ~KArcCache() override = default;

    void put(Key key, Value value) override
    {
        bool inGhost = checkGhostCaches(key);

        if ( !inGhost )
        {
            if (lruPart->put(key, value))
            {
                lfuPart->put(key,value);
            }
        } 
        else
        {
            lruPart->put(key, value);
        }
    }

    bool get(Key key, Value & value) override
    {
        checkGhostCaches(key);

        bool shouldTransform = false;

        if ( lruPart->get(key, value, shouldTransform) )
        {
            if ( shouldTransform)
            {
                lfuPart->put(key, value);
            }
            return true;
        }
        return lfuPart->get(key, value);
    }

    Value get(Key key) override
    {
        Value value{};
        get(key, value);
        return value;
    }


};    

}

