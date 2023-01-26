#include <time.h>
#include "tictoc_impl.h"

double MW_tic(void)
{
	return ((double) clock());
}

double MW_toc(double start)
{
    return (clock() - start) / CLOCKS_PER_SEC;
}
