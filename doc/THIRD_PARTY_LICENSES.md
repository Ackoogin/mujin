# Third-Party License Notices

This document lists all third-party dependencies used by AME, their licenses, and the compliance obligations that apply when distributing this software.

**Audit date:** 2026-04-01  
**Auditor:** Automated license compliance check

---

## Summary

All dependencies use **permissive open-source licenses**. There are no GPL, LGPL, AGPL, or other copyleft licenses in the dependency tree. The project is safe for closed-source and commercial deployment, subject to the attribution requirements described below.

| Dependency | Version | License | Type |
|---|---|---|---|
| BehaviorTree.CPP | 4.6.2 | MIT | Permissive |
| LAPKT | Devel2.0 | MIT | Permissive |
| nlohmann/json | v3.11.3 | MIT | Permissive |
| Google Test | v1.14.0 | BSD 3-Clause | Permissive |
| websocketpp | 0.8.2 | BSD 3-Clause | Permissive |
| Asio (standalone) | asio-1-28-0 | Boost Software License 1.0 | Permissive |
| pybind11 | v2.11.1 | BSD 3-Clause | Permissive (optional, off by default) |
| SQLite3 | 3.40.1 | Public Domain | N/A |
| dearpygui | ≥1.11 | MIT | Permissive (dev tooling only) |
| websocket-client | ≥1.6 | Apache-2.0 | Permissive (dev tooling only) |

---

## 1. BehaviorTree.CPP

**Version:** 4.6.2  
**Source:** https://github.com/BehaviorTree/BehaviorTree.CPP  
**License:** MIT

```
Copyright (c) 2019 Davide Faconti
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

**Compliance requirement:** Include the above copyright and permission notice in all copies or substantial portions of the software (source distributions, documentation, or a bundled notices file).

---

## 2. LAPKT (Lightweight Automated Planning Toolkit)

**Version:** Devel2.0  
**Source:** https://github.com/LAPKT-dev/LAPKT-public  
**License:** MIT (declared in per-file headers; no standalone LICENSE file on this branch)

```
Copyright 2022
Miquel Ramirez <miquel.ramirez@unimelb.edu.au>
Nir Lipovetzky <nirlipo@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files
(the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject
to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

**Compliance requirement:** Include the above copyright and permission notice in all copies or distributions that include LAPKT-derived compiled objects. Note: LAPKT's `Devel2.0` branch has no standalone `LICENSE` file — the notice appears only in individual source file headers. This document serves as the consolidated notice.

---

## 3. nlohmann/json

**Version:** v3.11.3  
**Source:** https://github.com/nlohmann/json  
**License:** MIT

```
MIT License

Copyright (c) 2013-2022 Niels Lohmann

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

**Compliance requirement:** Include the above copyright and permission notice in distributions.

---

## 4. Google Test

**Version:** v1.14.0  
**Source:** https://github.com/google/googletest  
**License:** BSD 3-Clause  
**Usage:** Test builds only; not linked into shipped binaries.

```
Copyright 2008, Google Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of Google Inc. nor the names of its contributors may be
  used to endorse or promote products derived from this software without
  specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```

**Compliance requirement:** BSD 3-Clause attribution required if test binaries are redistributed. The third clause prohibits using "Google Inc." or contributor names to endorse derived products without written permission.

---

## 5. websocketpp

**Version:** 0.8.2  
**Source:** https://github.com/zaphoyd/websocketpp  
**License:** BSD 3-Clause  
**Usage:** Only built when `AME_FOXGLOVE=ON` (default ON).

```
Copyright (c) 2014, Peter Thorson. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the WebSocket++ Project nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL PETER THORSON BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```

**Note:** websocketpp bundles sub-components with their own permissive licenses (Base64, SHA1, MD5, UTF-8 validator). All are permissive (BSD/MIT/Aladdin-style); none are copyleft.

**Compliance requirement:** Retain copyright notice and reproduce in binary documentation. The "WebSocket++ Project" name may not be used to endorse derived products.

---

## 6. Asio (standalone)

**Version:** asio-1-28-0  
**Source:** https://github.com/chriskohlhoff/asio  
**License:** Boost Software License 1.0 (BSL-1.0)  
**Usage:** Only built when `AME_FOXGLOVE=ON` (default ON).

```
Boost Software License - Version 1.0 - August 17th, 2003

Copyright (c) 2003-2023 Christopher M. Kohlhoff (chris at kohlhoff dot com)

Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of the software and accompanying documentation covered by
this license (the "Software") to use, reproduce, display, distribute,
execute, and transmit the Software, and to prepare derivative works of the
Software, and to permit third-parties to whom the Software is furnished to
do so, all subject to the following:

The copyright notices in the Software and this entire statement, including
the above license grant, this restriction and the following disclaimer,
must be included in all copies of the Software, in whole or in part, and
all derivative works of the Software, unless such copies or derivative
works are solely in the form of machine-executable object code generated by
a source language processor.
```

**Compliance requirement:** Include the notice in source distributions and documentation. Notably, BSL-1.0 does **not** require the notice in compiled binary-only distributions (machine-executable object code) — this is more permissive than MIT/BSD in that respect.

---

## 7. pybind11

**Version:** v2.11.1  
**Source:** https://github.com/pybind/pybind11  
**License:** BSD 3-Clause  
**Usage:** Optional; only built when `AME_BUILD_PYTHON=ON` (default OFF).

```
Copyright (c) 2016 Wenzel Jakob <wenzel.jakob@epfl.ch>, All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

**Compliance requirement:** Retain copyright notice; reproduce in binary documentation if Python bindings are distributed. Contributor names may not be used for endorsement.

---

## 8. SQLite

**Version:** 3.40.1  
**Source:** https://www.sqlite.org  
**License:** Public Domain  
**Usage:** Via Conan (`conanfile.txt`).

SQLite is in the public domain. All code has been explicitly dedicated to the public domain by the authors, who have signed affidavits to that effect.

**Compliance requirement:** None. No copyright notice, license text, or credit is required. Attribution is appreciated but legally unnecessary.

---

## 9. dearpygui (Python dev tooling)

**Version:** ≥1.11  
**Source:** https://github.com/hoffstadt/DearPyGui  
**License:** MIT  
**Usage:** Developer visualisation tool (`tools/devenv/`); not part of the deployed system.

**Compliance requirement:** Include MIT copyright notice if the dev tool is redistributed. No obligation for the core AME system.

---

## 10. websocket-client (Python dev tooling)

**Version:** ≥1.6  
**Source:** https://pypi.org/project/websocket-client/  
**License:** Apache License 2.0  
**Usage:** Developer tooling (`tools/devenv/`); not part of the deployed system.

Apache-2.0 is a permissive licence. Key points:
- Retain all copyright, patent, trademark, and attribution notices in source distributions.
- If a `NOTICE` file is present in the package, reproduce its contents in derivative distributions.
- Includes an explicit patent grant.
- Includes a **patent retaliation clause**: initiating patent litigation against the work terminates your patent licence automatically. This is a defensive mechanism, not a use restriction.
- Apache-2.0 is **incompatible with GPLv2** (relevant only if combining with GPLv2 components — there are none here).

**Compliance requirement:** Retain notices if redistributing the tool. No obligation for the core AME system.

---

## ROS2 Stack (optional integration)

**Usage:** `ros2/` subdirectory only; built separately.  
**License:** Apache License 2.0 (all core ROS2 packages).

Apache-2.0 is permissive and compatible with this project's dependency stack. Attribution requirements as described for websocket-client above apply if ROS2 binaries are redistributed.

---

## Distribution Checklist

When distributing AME binaries or source packages, ensure the following:

- [ ] This file (`doc/THIRD_PARTY_LICENSES.md`) is included in the distribution.
- [ ] For source distributions: all original license headers in LAPKT source files are preserved.
- [ ] The `AME_FOXGLOVE=ON` (default) build includes websocketpp and Asio — their notices are covered above.
- [ ] The `AME_BUILD_PYTHON=ON` build includes pybind11 — its notice is covered above.
- [ ] Google Test is test-only and not linked into shipped binaries; no additional action needed for binary-only releases.
- [ ] SQLite: no action required (public domain).
- [ ] Python dev tools (`tools/devenv/`) are not part of the core deployed system; their licenses apply only if those tools are redistributed.

---

## Tooling References (not bundled)

The following tools are referenced in planning documentation but are **not bundled** in this repository and are **not build dependencies**. They carry GPL or other licences that would impose obligations if included:

| Tool | License | Status |
|---|---|---|
| Fast Downward | GPL-3.0+ | Not included; referenced for evaluation only |
| TFD (Temporal Fast Downward) | GPL | Not included |
| LPRPG-P | GPL | Not included |
| PRISM | GPL | Not included |
| Storm | GPL | Not included |
| Scorpion | GPL | Not included |
| ENHSP | LGPL | Not included |

**Action required if any of the above are added as dependencies:** A legal review must be conducted before integrating any GPL or LGPL component, as this may impose copyleft obligations on the entire AME codebase.
