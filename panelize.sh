#!/bin/bash
kikit panelize --layout 'grid; rows: 1; cols: 2; hspace: 5mm' --source 'tolerance: 15mm' --tabs annotation --cuts 'mousebites; drill: 0.5mm; spacing: 0.9mm; offset: -0.2mm; prolong: 0.9mm' --framing 'railstb; width: 8mm; space: 3mm; fillet: 1mm' --post 'millradius: 1mm' --fiducials '4fid; hoffset: 5mm; voffset: 2.5mm; coppersize: 1mm; opening: 3mm;' rusty-tracker.kicad_pcb panel.kicad_pcb
git rev-parse --short HEAD | xargs -I % sed -i 's/{GITHASH}/%/g' panel.kicad_pcb
