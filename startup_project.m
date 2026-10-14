% Настройка окружения корневого проекта
disp('Инициализация temp_sensor_system...');

% Общие параметры системы
assignin('base', 'TS_SAMPLE_TIME', 0.01);
assignin('base', 'ADC_MIN', 0);
assignin('base', 'ADC_MAX', 100);

disp('Окружение готово.');