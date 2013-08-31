#include "errors.h"

#include <stdlib.h>

const char *errorStrings[NUM_ERRORS] = {
    "Success",
    "Argument Error",
    "Acc Read Error",
    "Acc Write Error",
    "Gyro Write Error",
    "Gyro Read Error",
    "Alt Write Error",
    "Alt Read Error"
};

const char *getErrorString(int error) {
    if (error < NUM_ERRORS)
        return errorStrings[error];
    return NULL;
}
