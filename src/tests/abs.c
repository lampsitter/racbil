#include "../assists.h"
#include "../common.h"
#include <assert.h>
#include <stdlib.h>

#define ASSERT_EQ(a, b) assert(fabsf(a - b) < EPSILON)
#define ASSERT_NEQ(a, b) assert(fabsf(a - b) > EPSILON)

int main(void)
{

    Abs abs = abs_new(-0.1f, 2.0f);
    const float pressure = 1000.0f;

    // Below active abs velocity
    ASSERT_EQ(abs_pressure(&abs, pressure, 1.0f, -0.3f), pressure);

    ASSERT_EQ(abs_pressure(&abs, pressure, 5.0f, -0.3f), 0.0f);
    ASSERT_EQ(abs_pressure(&abs, pressure, 5.0f, 0.3f), pressure);

    // Below active slip ratio (inactive)
    ASSERT_EQ(abs_pressure(&abs, pressure, 5.0f, -0.1f), pressure);

    // disabled
    abs.is_enabled = false;
    ASSERT_EQ(abs_pressure(&abs, pressure, 5.0f, -0.3f), pressure);

    return 0;
}
