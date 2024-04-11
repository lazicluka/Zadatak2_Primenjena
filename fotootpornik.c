#include <p30fxxxx.h>
#include "fotootpornik.h"

unsigned int sirovi;

int fotootpornik(unsigned int sirovi)
{
    if (sirovi < 1200)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// funkcija koja se koristi za osvetljenje fotoR,moze se menjati u zavisnosti od okolinog svetla