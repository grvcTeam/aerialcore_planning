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
