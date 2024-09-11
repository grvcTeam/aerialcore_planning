%% Iniciar ROS
% Definir la dirección IP del maestro ROS y el puerto (cambiar si es necesario)
ipAddress = 'http://localhost:11311';  % Dirección del maestro ROS
rosinit(ipAddress);  % Iniciar conexión con el maestro ROS

%% Crear un publicador en el tema '/chatter' con el tipo de mensaje 'std_msgs/String'
pub = rospublisher('/chatter', 'std_msgs/String');

% Crear un mensaje de tipo 'std_msgs/String'
msg = rosmessage(pub);
msg.Data = '¡Hola desde MATLAB!';

% Publicar el mensaje en el tema
send(pub, msg);

%% Crear un suscriptor al tema '/chatter' con el tipo de mensaje 'std_msgs/String'
sub = rossubscriber('/chatter', 'std_msgs/String');

% Recibir un mensaje del tema
receivedMsg = receive(sub, 10);  % Esperar hasta 10 segundos para recibir un mensaje
disp(receivedMsg.Data);  % Mostrar el contenido del mensaje recibido

%% Finalizar la conexion con ROS
rosshutdown;

%%
% Iniciar script de inicialización para mensajes de ROS en MATLAB

% Paso 1: Obtener la ruta del ROS_PACKAGE_PATH desde el entorno de ROS
rosPackagePath = getenv('ROS_PACKAGE_PATH');

% Verificar si se ha encontrado el ROS_PACKAGE_PATH
if isempty(rosPackagePath)
    error('No se ha encontrado la variable de entorno ROS_PACKAGE_PATH. Asegúrate de que ROS esté configurado correctamente.');
end

% Separar las posibles múltiples rutas en ROS_PACKAGE_PATH
packagePaths = strsplit(rosPackagePath, pathsep);

%% Paso 2: Buscar la ruta del paquete que contiene los mensajes personalizados
% Modifica el nombre del paquete a buscar según tus necesidades
packageName = 'human_aware_collaboration_planner'; % Cambia esto al nombre de tu paquete con mensajes personalizados
packagePath = '';

% Iterar sobre las rutas para encontrar la del paquete deseado
for i = 1:length(packagePaths)
    % Intentar encontrar el paquete usando rospack find (requiere que ROS esté en el sistema PATH)
    [status, cmdout] = system(['rospack find ', packageName]);
    
    % Verificar si se encontró la ruta del paquete
    if status == 0
        packagePath = strtrim(cmdout); % Remover espacios en blanco
        break;
    end
end

% Verificar si se encontró el paquete
if isempty(packagePath)
    error(['No se ha encontrado el paquete ', packageName, '. Asegúrate de que el paquete está en tu workspace de ROS y configurado correctamente.']);
end

%% Paso 3: Generar los mensajes personalizados
try
    disp('Generando mensajes personalizados de ROS para MATLAB...');
    rosgenmsg(packagePath);
catch ME
    error('Error al generar los mensajes personalizados de ROS: %s', ME.message);
end

%% Paso 4: Agregar la carpeta generada al path de MATLAB
generatedPath = fullfile(packagePath, 'matlab_msg_gen', 'ros', '+mission_planner'); % Ajusta según sea necesario
addpath(generatedPath);
savepath; % Guarda el nuevo path para futuras sesiones

disp('Mensajes personalizados de ROS configurados correctamente en MATLAB.');
