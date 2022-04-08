Binary executables used for web scraping.  
A system call to CURL is made and a connection to the site is established.  
Contents are then dumped into a temporary html file, which is further read and parsed.  
Processed contents are saved into files under `docs`.  

Note: The executable is written in C++, and is not intended to be run without GitHub Actions or on a local device.
