define hook-quit
  set confirm off
end
target extended-remote localhost:3333

b swv_trap_line
commands
  silent
  printf "%s\n", swv_buf
  c
end
r
