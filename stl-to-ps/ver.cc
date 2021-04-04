#define a(x) b(x)
#define b(x) #x
static_assert(__cplusplus > 201103L, a(__cplusplus));

