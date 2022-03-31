#ifndef SENSOR_GLOBAL_H
#define SENSOR_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(SENSOR_LIBRARY)
#  define SENSOR_EXPORT Q_DECL_EXPORT
#else
#  define SENSOR_EXPORT Q_DECL_IMPORT
#endif

#endif // SENSOR_GLOBAL_H
