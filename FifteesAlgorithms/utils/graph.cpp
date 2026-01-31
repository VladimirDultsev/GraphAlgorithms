#include "graph.h"
#include <iostream>
#include <vector>
#include <__ostream/basic_ostream.h>

namespace Graph
{
    unsigned long long int start = 0;
    unsigned long long int finish = 1311768467463790320ULL;
    unsigned long long int Start = 0;
    unsigned long long int threshold = 15000000;
    bool error = false;

    unsigned long long int swap(unsigned long long int st, const unsigned short i, const unsigned short zeroPos)
    {
        unsigned long long int num = (static_cast<unsigned long long int>(15)<<4*(15 - i)&st)>>4*(15 - i);
        st -= (static_cast<unsigned long long int>(15)<<4*(15 - i))&st;
        st += num<<4*(15 - zeroPos);
        return st;
    }

    void CalcVariants(unsigned long long int f, std::vector<unsigned long long int>& ls)
    {
        unsigned short zeroPos = 0;
        ls.clear();
        for (unsigned short i = 0; i < 16; i++)
        {
            if((static_cast<unsigned long long int>(15)<<4*i&f)>>4*i == 0)// если ноль - это ноль
            {
                zeroPos = 15 - i;
                break;
            }
        }
        if (zeroPos - 4 > 0)
        {
            ls.push_back(swap(f, zeroPos - 4, zeroPos));
        }
        if (zeroPos + 4 <= 15)
        {
            ls.push_back(swap(f, zeroPos + 4, zeroPos));
        }
        if (zeroPos != 0 && zeroPos != 4 && zeroPos != 8 && zeroPos != 12)
        {
            ls.push_back(swap(f, zeroPos - 1, zeroPos));
        }
        if (zeroPos != 3 && zeroPos != 7 && zeroPos != 11 && zeroPos != 15)
        {
            ls.push_back(swap(f, zeroPos + 1, zeroPos));
        }
    }

    unsigned long long int fieldToLong(const std::vector<unsigned short>& f)
    {
        unsigned long long int Res = 0;
        for (int i = 0; i < 16; ++i)
        {
            Res |= static_cast<unsigned long long int>(f[i]) << (60 - i * 4);
        }
        return Res;
    }

    void longToField(unsigned long long int num, std::vector<unsigned short>& f)
    {
        if (f.size() != 16) f.resize(16);
        for (int i = 0; i < 16; ++i)
        {
            f[15 - i] = static_cast<unsigned short>((num >> i * 4) % 16);
        }
    }

    void print(unsigned long long int l)
    {
        std::vector<unsigned short> field;
        Graph::longToField(l, field);
        for (int i = 0; i < 16; i++)
        {
            if (i % 4 == 0)
            {
                std::cout << std::endl;
            }
            std::cout << field[i] << " ";
        }
        std::cout << std::endl;
    }

    bool amountOfInversions(std::vector<unsigned short>& f)
    {
        int amountOfInversions = 0;
        for (int i = 0; i < 16; i++)
        {
            for (int j = 0; j < 16 - i; j++)
            {
                if (f[i + j] < f[i] && f[i] != 0)
                {
                    amountOfInversions++;
                }
            }
        }
        if (amountOfInversions % 2 == 0)
        {
            return true;
        }
        return false;
    }
}