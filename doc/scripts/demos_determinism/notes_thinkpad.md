Results:

| Demo                             | Valid multiccd="enable" | Valid multiccd="disable" | Notes                                                                                                                  |
|----------------------------------|-------------------------|--------------------------|------------------------------------------------------------------------------------------------------------------------|
| ff0bb5b5b80948cfaa9e4a4a3fb42b18 | +                       | -                        |                                                                                                                        |
| 1d738d73cead4e9f8849fd6a90bb8511 | +                       | -                        |                                                                                                                        |
| f539e0f5787b4a74949e72a8a206e6bc | -                       | -                        | **multiccd off**: cup drops to floor<br/>**multiccd on**: cup stays on the counter                                     |
| b6b8921f56364bfebf87bab3e89512b9 | -                       | -                        | **multiccd off**: grabs spoon, but it drops during the task execution<br/>**multiccd on**: misses the spoon completely |


Command:
```shell
lscpu           # CPU details
lspci | grep -i vga    # GPU details
free -h         # RAM info
```

Output:
```text
Architecture:             x86_64
  CPU op-mode(s):         32-bit, 64-bit
  Address sizes:          39 bits physical, 48 bits virtual
  Byte Order:             Little Endian
CPU(s):                   8
  On-line CPU(s) list:    0-7
Vendor ID:                GenuineIntel
  Model name:             Intel(R) Core(TM) i5-8350U CPU @ 1.70GHz
    CPU family:           6
    Model:                142
    Thread(s) per core:   2
    Core(s) per socket:   4
    Socket(s):            1
    Stepping:             10
    CPU max MHz:          3600,0000
    CPU min MHz:          400,0000
    BogoMIPS:             3799.90
    Flags:                fpu vme de pse tsc msr pae mce cx8 apic sep mtrr pge mca cmov pat pse36 clflush dts acpi mmx fxsr sse sse2 ss ht tm pbe syscall nx pdpe1gb rdtscp lm constant_tsc art arch_perfmon pebs bts rep_good nopl xtopology nonstop_tsc cpuid aperfmperf pni pclmulqdq dtes64 monitor ds_cpl vmx smx est tm2 ssse3 sdbg fma cx16 xtpr pdcm pcid sse4_1 sse4_2 x2apic movbe popcnt tsc_deadline_timer aes xsave avx
                           f16c rdrand lahf_lm abm 3dnowprefetch cpuid_fault epb pti ssbd ibrs ibpb stibp tpr_shadow flexpriority ept vpid ept_ad fsgsbase tsc_adjust bmi1 avx2 smep bmi2 erms invpcid mpx rdseed adx smap clflushopt intel_pt xsaveopt xsavec xgetbv1 xsaves dtherm ida arat pln pts hwp hwp_notify hwp_act_window hwp_epp vnmi md_clear flush_l1d arch_capabilities
Virtualization features:
  Virtualization:         VT-x
Caches (sum of all):
  L1d:                    128 KiB (4 instances)
  L1i:                    128 KiB (4 instances)
  L2:                     1 MiB (4 instances)
  L3:                     6 MiB (1 instance)
NUMA:
  NUMA node(s):           1
  NUMA node0 CPU(s):      0-7
Vulnerabilities:
  Gather data sampling:   Mitigation; Microcode
  Itlb multihit:          KVM: Mitigation: VMX disabled
  L1tf:                   Mitigation; PTE Inversion; VMX conditional cache flushes, SMT vulnerable
  Mds:                    Mitigation; Clear CPU buffers; SMT vulnerable
  Meltdown:               Mitigation; PTI
  Mmio stale data:        Mitigation; Clear CPU buffers; SMT vulnerable
  Reg file data sampling: Not affected
  Retbleed:               Mitigation; IBRS
  Spec rstack overflow:   Not affected
  Spec store bypass:      Mitigation; Speculative Store Bypass disabled via prctl
  Spectre v1:             Mitigation; usercopy/swapgs barriers and __user pointer sanitization
  Spectre v2:             Mitigation; IBRS; IBPB conditional; STIBP conditional; RSB filling; PBRSB-eIBRS Not affected; BHI Not affected
  Srbds:                  Mitigation; Microcode
  Tsx async abort:        Mitigation; TSX disabled
00:02.0 VGA compatible controller: Intel Corporation UHD Graphics 620 (rev 07)
               total        used        free      shared  buff/cache   available
Mem:            15Gi       5,5Gi       1,9Gi       530Mi       8,1Gi       9,1Gi
Swap:          2,0Gi        18Mi       2,0Gi
```
