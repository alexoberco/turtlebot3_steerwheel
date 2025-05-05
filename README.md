## Colocación de archivos  
Coloca **Dockerfile**, **.dockerignore** y **entrypoint.sh** en la raíz de tu workspace (el directorio desde el que ejecutarás \`docker build\`) para que Docker los reconozca como parte del contexto de construcción.  

## Construcción de la imagen  
Desde la carpeta raíz de tu workspace, ejecuta:  
```bash
docker build -t ros2-humble-turtlebot3 .
```  
## Ejecución del contenedor  
Para exponer todos los dispositivos de entrada y los datos de udev, lanza el contenedor con:  
```bash
docker run -it --rm \\\\  
  --privileged \\\\  
  --net=host \\\\  
  -v /dev/input:/dev/input \\\\  
  -v /run/udev/data:/run/udev/data \\\\  
  ros2-humble-turtlebot3:latest
```  
## Uso dentro del contenedor  
Una vez dentro de la sesión interactiva del contenedor, carga tu workspace y lanza el nodo de teleoperación con:  
```bash
source install/setup.bash  
ros2 launch turtle_codes steerwheel.launch.py
```  
