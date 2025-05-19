#pragma once

namespace KamaCache
{
    template <typename Key, typename Value>
    class  KICachePolicy
    {
    public:
        virtual ~ KICachePolicy() {};
        virtual void put(Key key, Value value) = 0;
        virtual bool get(Key key, Value & value) = 0;
        virtual Value get(Key key) = 0;
    };
    

    
}