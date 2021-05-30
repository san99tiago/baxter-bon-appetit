# MATLAB METADATA

## Things to keep in mind for the correct Baxter simulation usage

* Utilizar el archivo llamado 'Exe_Sim_Baxter.m', estando en el directorio 
'RETO BAXTER', donde se encuentran todos los scripts.

* En ese archivo solo se tienen que cambiar las condiciones iniciales y 
finales de las cantidades asociadas a la traslación y a la orientación, 
y tambien ingresar un valor de tiempo para la ejecucion de la trayectoria, 
junto con el periodo de muestreo de la simulación.

* Después de esto solo se corre el programa y la magia comienza...

## Important remarks

These scripts are not developed with clean-code architecture or the best 
practices. However, they are just functional tests to validate important 
Baxter features directly on Matlab.<br>

**The important code aspects are inside the "python" directory**