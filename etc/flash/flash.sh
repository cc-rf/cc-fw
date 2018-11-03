 #!/bin/bash -e
 d=$(dirname $(readlink -f "${0}"))
 
 cd "${d}"
 openocd -f "${d}/cloudchaser.cfg"
