#include <iostream>
#include "KLruCache_copy.h"


int main()
{
    using namespace std;
    using namespace Kamaspace;
    LruCache<int, int> test (5);

    for ( int index = 0; index <= 6; index++)
    {
        test.put(index, index);
    }
    int result = 0;
    for (int index = 0; index <= 6; index++)
    {
        result = test.get(index);
        cout << result <<" " ;
    }

    return 0;
}