#include "eso.h"

static inline void eso_set_w(ESOState* s, float w)
	{
    s->beta1 = 2.0f * w;
    s->beta2 = w * w;
	}

void ESO_Init(ESOState* s)
	{
    if (!s) return;
    s->z1 = s->z2 = s->error = 0.0f;
    s->T  = 0.001f;
    s->b0 = 1000.f;
    eso_set_w(s, 80.f);
	}

void ESO_InitWithParams(ESOState* s, float Ts, float w, float b0)
	{
    if (!s) return;
    s->z1 = s->z2 = s->error = 0.0f;
    s->T  = Ts;
    s->b0 = b0;
    eso_set_w(s, w);
	}

void ESO_Step(ESOState* s, float u_volt, float y_meas)
	{
    if (!s) return;
    float e = y_meas - s->z1;
    s->error = e;
    float T = s->T;
    s->z1 += T * ( s->z2 + s->b0 * u_volt + s->beta1 * e );
    s->z2 += T * ( s->beta2 * e );
	}

void ESO_ALLInit(ESOState* arr, int count,
                  const float* Ts, const float* w, const float* b0)
{
    if (!arr || count <= 0) return;
    for (int i = 0; i < count; ++i)
		{
        if (Ts && w && b0) ESO_InitWithParams(&arr[i], Ts[i], w[i], b0[i]);
        else               ESO_Init(&arr[i]);
    }
}



