README version 4/26/2018

The purpose of the "clean" iterations of the baro and GPS main.c files is to clean up our testing
environment and to take the test code we have written and put it into an older version of main()
where the initializations below main are still the direct outputs from TrueSTUDIO. The clean versions
of the GPS and baro main files are untouched since creation below main(). They are as TrueSTUDIO created them.

If you wish to make changes to ANYTHING below the main() function in the clean versions of the programs,
please remove the "clean" from the filename. Clean denotes fresh intialization for everything.

- main_gps_clean:
Added GPS test code to main
Updated functions above main() [identical to main_baro_clean]
Cleaned up comments (didn't delete anything, only made more legible)
Commented out parser() until Glenn gives seal of approval
Removed main() while loop pseudocode logic (still have it saved on G Drive)

- main_baro_clean
Added baro test code to main
Updated functions above main() [identical to main_gps_clean]
Cleaned up comments (didn't delete anything, only made more legible)
Commented out parser() until Glenn gives seal of approval
Removed main() while loop pseudocode logic (still have it saved on G Drive)

- main_original_backup
This is the original code I started with before doing anything.