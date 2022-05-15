#include "../common.h"
#include <assert.h>

int main(void)
{

    Table table = table_with_capacity(2, 3);

    table.x[0] = 0.0;
    table.x[1] = 2.0;

    table.y[0] = -10.0;
    table.y[1] = 0.0;
    table.y[2] = 10.0;

    table.z[0][0] = 20.0;
    table.z[0][1] = 10.0;
    table.z[0][2] = 20.0;

    table.z[1][0] = 10.0;
    table.z[1][1] = 10.0;
    table.z[1][2] = 40.0;

    assert(table_lookup(&table, 1.0, 5.0) == 20.0);
    assert(table_lookup(&table, 1.0, 10.0) == 30.0);

    table_free(&table);

    return 0;
}
