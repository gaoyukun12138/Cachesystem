#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <thread>
#include "KICachePolicy.h"




namespace Kamaspace
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
        node * pre_;
        node * next_;
        Node() : freq_(1), pre_(nullptr), next_(nullptr)
        {};
        Node(Key key, Value value) : key_(key), value(value_), freq_(1)
        {
            pre_ = nullptr;
            next_ = nullptr;
        }
    };

private:
    int ListFreq;
    std::shared_ptr<Node> dummyhead;
    std::shared_ptr<Node> dummytail;

public:
    FreqList(int n) : ListFreq(n)
    {
        dummyhead = new Node(-1, -1);
        dummytail = new Node(-1, -1);
        dummyhead->next_ = dummytail;
        dummytail->pre_ = dummyhead;
    }

    bool IsEmpty()
    {
        return dummyhead->next_ == dummytail;
    }

    void RemoveNode( std::shared_ptr<Node> node )
    {
        if ( !node || !dummyhead || !dummytail)
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

    void AddNode(std::shared_ptr<Node> node)
    {
        if ( !node || !dummyhead || !dummytail)
        {
            return;
        }
        std::shared_ptr<Node> next = dummyhead->next_;
        dummyhead->next_ = node;
        next->pre_ = node;
        node->pre_ = dummyhead;
        node->next_ = next;
    }

    std::shared_ptr<Node> GetFirstNode()
    {
        return dummytail->pre_;
    }
    
    friend class KLfuCache<Key, Value>;
};


template<typename Key, typename Value>
class KLfuCache : public KICachePolicy
{
public:
    using Node = typename FreqList<Key, Value>::Node;
    using NodePtr = std::shared_ptr<Node>;
    using NodeMap = std::unordered_map<Key, NodePtr>;
private:
    size_t capacity_;
    int MinFreq_;
    int CurAverageNum_;
    int MaxAverageNum_;
    int CurTotalNum_;
    NodeMap NodeMap_;
    std::mutex Mutex_;
    std::unordered_map<int, FreqList<Key, Value>* >  FreqToFreqList_;

    void RemoveFromList(NodePtr node)
    {
        int Freq = node->freq_;
        FreqToFreqList_[Freq]->RemoveNode(node);

    }

    void AddToList(NodePtr node)
    {
        if ( !node )
        {
            return;
        }

        int Freq = node->freq_;
        if ( FreqToFreqList_.find(Freq) == FreqToFreqList_.end() )
        {
            FreqToFreqList_[Freq] = new FreqList<Key, Value> (Freq);
        }

        FreqToFreqList_[Freq]->AddNode(node);
    }


    void UpdateMinFreqNum()
    {
        MinFreq_ = INT8_MAX;
        for ( const auto & iter : FreqToFreqList_)
        {
            if ( iter->second->IsEmpty() )
            {
                MinFreq_ = std::min(MinFreq_, iter->first);
            }
        }

        if ( MinFreq_ ==  INT8_MAX)
        {
            MinFreq_ = 1;
        }
    }

    void HandleOverMaxAverageNum()
    {
        if ( NodeMap_.size() == 0)
        {
            return;
        }

        for ( auto iter : NodeMap_ )
        {
            RemoveFromList(iter->second);
            iter->second->freq_ -= MaxAverageNum_ / 2;
            if (iter->second->freq_ < 1)
            {
                iter->second->freq_ = 1;
            }

            AddToList(iter->second);
        }

        UpdateMinFreqNum();

    }

    void AddFreqNum()
    {
        CurTotalNum_++;
        if ( NodeMap_.size() == 0)
        {
            CurAverageNum_ = 0;
        }

        CurAverageNum_ = CurTotalNum_ / NodeMap_.size();
        if ( CurAverageNum_ > MaxAverageNum_)
        {
            HandleOverMaxAverageNum();
        }
    }

    void GetInterNode( NodePtr node, Value & value)
    {
        value = node->value_;
        RemoveFromList(node);
        node->freq_++;
        AddToList(node);

        if ( node->freq_-1 == MinFreq_ && FreqToFreqList_[MinFreq_]->IsEmpty() )
        {
            MinFreq_++;
        }

        AddFreqNum();
    }

    void ReFreashFreqNum(int num)
    {
        CurTotalNum_ -= num;

        if ( NodeMap_.size() == 0)
        {
            CurAverageNum_ = 0;
        }
        else
        {
            CurAverageNum_ = CurTotalNum_ / NodeMap_.size();
        }
    }

    void KickOut()
    {
        NodePtr cur = FreqToFreqList_[MinFreq_]->GetFirstNode();
        RemoveFromList(cur);
        ReFreashFreqNum(cur->freq_);
        NodeMap_.erase(Key);

    }

    void PutInterNode(Key key, Value value)
    {
        if ( NodeMap_.size() == capacity_ )
        {
            KickOut();
        }

        NodePtr newnode = std::make_shared<Node> (key, value);
        NodeMap_[key] = newnode;
        AddToList(newnode);
        AddFreqNum();

        MinFreq_ = std::min(MinFreq_, 1);


    }


public:
    KLfuCache(std::size_t capacity, int MaxAverageNum)
    : capacity_(capacity), MaxAverageNum_(MaxAverageNum), MinFreq_(INT_MAX)
    {
        CurTotalNum_ = 0;
        CurAverageNum_ = 0;
    }


    void put(Key key, Value value)
    {
        if (capacity_ == 0)
        {
            return;
        }
        std::lock_guard<std::mutex> lock(Mutex_);
        if ( NodeMap_.find(key) != NodeMap_.end() )
        {
            NodePtr node = NodeMap_[key];
            node->value_ = value;
            GetInterNode(node, value);
            return;
        }

        PutInterNode(key, value);
    }

    bool get(Key key, Value & value)
    {
        std::lock_guard<std::mutex> lock(Mutex_);
        auto iter = NodeMap_.find(key);
        if ( iter != NodeMap_.end() )
        {
            GetInterNode(iter->second, value);
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

    void purge()
    {
        NodeMap_.clear();
        FreqToFreqList_.clear();
    }

};

template<typename Key, typename Value> 
class KHashLfuCache
{
private:
    std::size_t capacity_;
    int SliceNum;
    std::vector< std::unique_ptr<KLfuCache<Key, Value> > > LfuHashslices;

    std::size_t Hash( Key key)
    {
        std::hash<Key> hashfunc;
        return hashfunc(key);
    }

public:
    KHashLfuCache(size_t capacity, int SliceNum, int MaxAvergeNum)
    : capacity_(capacity), 
    SliceNum( SliceNum > 0 ? SliceNum :  std::thread::hardware_concurrency())
    {
        std::size_t Slicesize = capacity_ / SliceNum;
        for ( int i = 0; i < SliceNum; i++)
        {
            LfuHashslices[i].emplace_back(std::make_unique<KLfuCache<Key, Value> > (Slicesize, MaxAvergeNum));
        }
    }

    void put(Key key, Value value)
    {
        std::size_t SliceINdex = Hash(key) % SliceNum;
        LfuHashslices[SliceINdex]->put(key, value);
    }

    void get(Key key, Value & value)
    {
        std::size_t SliceIndex = Hash(key) % SliceNum;
        LfuHashslices[SliceIndex]->get(key, value);
    }

    void get(Key key)
    {
        Value value{};
        get(key, value);
        return value;
    }

    void purge()
    {
        for ( auto & iter : LfuHashslices )
        {
            iter->purge();
        }
    }

};

}