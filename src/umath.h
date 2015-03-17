#ifndef UMATH_H
#define UMATH_H
/**
 * Return true if the number is finite.
 */
template<class T>
inline bool uIsFinite(const T & value)
{
#if _MSC_VER
    return _finite(value) != 0;
#else
    return std::isfinite(value);
#endif
}

#endif // UMATH_H

