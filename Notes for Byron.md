Notes for Byron

# there are multiple definitions of mozziAnalogRead() in the Mozzi library itself.

The code compiles, but intellisense underlines any call to mozziAnalogRead() with red because it sees the redefinition and decides it's an error. This is a problem with Mozzi, not our code.

There's no errant #include <mozzi_analog.h> anywhere in your src/, include/, or lib/ directories. The problem is entirely inside Mozzi itself.
The chain is:
Mozzi.h
- MozziGuts.h (line 33): #include "mozzi_analog.h"   ← defines mozziAnalogRead
- MozziGuts.h (line 205): #include "internal/MozziGuts.hpp"
    - MozziGuts.hpp (line 15): #include "mozzi_analog.h"   ← header guard blocks re-inclusion
    - MozziGuts.hpp (line 304): defines mozziAnalogRead again, independently
The header guard on mozzi_analog.h prevents the file itself from being processed twice, but MozziGuts.hpp contains its own separate template definition of mozziAnalogRead at line 304, independent of the one in mozzi_analog.h. Both end up visible in the same translation unit.

Possible fixes seem to be either:
- Add a "C_Cpp.errorSquiggles": "disabled" entry to .vscode/settings.json. But I think that would kill all error squiggles, which I don't want. Maybe there's a way to do a targetted one just for this problem.
- File a bug against Mozzi 2.x pointing at the duplicate template in MozziGuts.hpp line 304 — that's where the redundant definition lives that should probably be removed or guarded.
- Just patch this in our own version of Mozzi, but that seems unneccesary.







Array stored in RAM:
RAM:   [=======   ]  72.8% (used 1491 bytes from 2048 bytes)
Flash: [=======   ]  68.6% (used 21064 bytes from 30720 bytes)

Fancy bit manipulation version:
RAM:   [=======   ]  72.5% (used 1485 bytes from 2048 bytes)
Flash: [=======   ]  68.8% (used 21124 bytes from 30720 bytes)

Finally realizing I should just store the array in flash:
RAM:   [=======   ]  72.5% (used 1485 bytes from 2048 bytes)
Flash: [=======   ]  68.6% (used 21064 bytes from 30720 bytes)