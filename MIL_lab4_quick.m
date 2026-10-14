% MIL_lab4_quick.m — MATLAB R2023b, лабораторная №4
% Проверяет 11 сценариев в корневой модели и сравнивает правильные ветви
% с прямой симуляцией независимых моделей датчиков.
% Запускать при открытом MATLAB Project temp_sensor_system.

proj = currentProject;
assert(strcmp(char(proj.Name), 'temp_sensor_system'), ...
    'Сначала откройте корневой MATLAB Project temp_sensor_system.');
root = char(proj.RootFolder);

mainModel = 'function_temp_sensor';
load_system(mainModel);

RequestedSelect = [1;1;1;2;2;2;3;3;0;4;255];
ADC_Value       = [0;50;100;0;50;100;50;100;50;50;50];
ExpectedInfo    = [1;1;1;2;2;2;3;3;0;0;0];
n = numel(RequestedSelect);
T = (0:n-1)' * 0.01;

% ВАЖНО: у SelectSensor в сгенерированном C-коде тип int8.
% Значение 255 не представимо в int8: int8(255) == 127.
AppliedSelect = double(int8(RequestedSelect));
MeasuredReal = nan(n, 1);
MeasuredInfo = nan(n, 1);
ExpectedReal = nan(n, 1);
Status = repmat("NOT_RUN", n, 1);

try
    ds = Simulink.SimulationData.Dataset;
    ds = addElement(ds, timeseries(int8(AppliedSelect), T), 'SelectSensor');
    ds = addElement(ds, timeseries(double(ADC_Value), T), 'ADC_Value');

    in = Simulink.SimulationInput(mainModel);
    in = setExternalInput(in, ds);
    in = setModelParameter(in, 'StopTime', sprintf('%.2f', T(end)), ...
        'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
    result = sim(in);
    outDS = result.get('yout');
    assert(isa(outDS, 'Simulink.SimulationData.Dataset'), ...
        'Не получен Dataset yout. Проверьте параметры Output в Model Settings.');
    assert(numElements(outDS) >= 2, ...
        'Ожидались два выходных порта Real_Value и Info_From_Sensor.');
    x = outDS.getElement(1).Values;
    y = outDS.getElement(2).Values;
    MeasuredReal = double(reshape(x.Data, [], 1));
    MeasuredInfo = double(reshape(y.Data, [], 1));
    assert(numel(MeasuredReal) == n && numel(MeasuredInfo) == n, ...
        'Количество отсчётов на выходах не равно 11.');
    Status(:) = "CHECK";
catch ME
    warning('Общая MIL-симуляция не завершилась: %s', ME.message);
    % Если одно граничное значение (например, log(0)) остановило расчёт,
    % пробуем каждый сценарий отдельно, сохраняя остальные результаты.
    for j = 1:n
        try
            tOne = [0; 0.01];
            inDS = Simulink.SimulationData.Dataset;
            inDS = addElement(inDS, timeseries(int8([AppliedSelect(j); ...
                AppliedSelect(j)]), tOne), 'SelectSensor');
            inDS = addElement(inDS, timeseries([ADC_Value(j); ...
                ADC_Value(j)], tOne), 'ADC_Value');
            one = Simulink.SimulationInput(mainModel);
            one = setExternalInput(one, inDS);
            one = setModelParameter(one, 'StopTime','0.01', ...
                'SaveOutput','on','OutputSaveName','yout','SaveFormat','Dataset');
            outOne = sim(one);
            yOne = outOne.get('yout');
            MeasuredReal(j) = double(yOne.getElement(1).Values.Data(end));
            MeasuredInfo(j) = double(yOne.getElement(2).Values.Data(end));
            Status(j) = "CHECK";
        catch MEone
            warning('Сценарий %d: %s', j, MEone.message);
            Status(j) = "MAIN_SIM_ERROR";
        end
    end
end

% Ожидаемые значения Real для ветвей 1–3 берём из независимых
% сенсорных моделей. Это проверяет интеграцию; ручные формулы из блоков
% датчиков при необходимости отдельно приводятся в отчёте.
sensorModels = {'temp_sensor_log', 'temp_sensor_exp', 'temp_sensor_square'};
for k = 1:3
    idx = find(RequestedSelect == k);
    tRef = (0:numel(idx)-1)' * 0.01;
    try
        m = sensorModels{k};
        load_system(m);
        refDS = Simulink.SimulationData.Dataset;
        refDS = addElement(refDS, ...
            timeseries(double(ADC_Value(idx)), tRef), 'Measure');
        refIn = Simulink.SimulationInput(m);
        refIn = setExternalInput(refIn, refDS);
        refIn = setModelParameter(refIn, ...
            'StopTime', sprintf('%.2f', tRef(end)), ...
            'SaveOutput', 'on', 'OutputSaveName', 'yout', 'SaveFormat', 'Dataset');
        refOut = sim(refIn);
        refY = refOut.get('yout');
        refSig = refY.getElement(1).Values;
        refVals = double(reshape(refSig.Data, [], 1));
        assert(numel(refVals) == numel(idx), ...
            'Количество отсчётов эталонной модели не совпадает.');
        ExpectedReal(idx) = refVals;
    catch ME
        warning('Отдельный датчик %s: %s', sensorModels{k}, ME.message);
        if any(Status(idx) == "CHECK")
            Status(idx) = "REFERENCE_ERROR";
        end
    end
end

% Для default обе ожидаемые величины равны нулю.
ExpectedReal(RequestedSelect == 0 | RequestedSelect == 4 | ...
    RequestedSelect == 255) = 0;

for i = 1:n
    if Status(i) ~= "CHECK"
        continue;
    end
    infoOK = isequal(MeasuredInfo(i), ExpectedInfo(i));
    realOK = (isequaln(MeasuredReal(i), ExpectedReal(i)) || ...
        (isfinite(MeasuredReal(i)) && isfinite(ExpectedReal(i)) && ...
        abs(MeasuredReal(i)-ExpectedReal(i)) <= ...
        1e-8 * max(1,abs(ExpectedReal(i)))));
    if infoOK && realOK
        if isnan(MeasuredReal(i))
            Status(i) = "BOUNDARY_NAN";
        elseif isinf(MeasuredReal(i))
            Status(i) = "BOUNDARY_INF";
        else
            Status(i) = "OK";
        end
    else
        Status(i) = "FAIL";
    end
end

Scenario = (1:n)';
results = table(Scenario, RequestedSelect, AppliedSelect, ADC_Value, ...
    ExpectedInfo, ExpectedReal, MeasuredInfo, MeasuredReal, Status);

% Файлы отчёта сохраняются рядом с temp_sensor_system, не внутрь Git.
outputFile = fullfile(fileparts(root), 'MIL_results_lab4.csv');
writetable(results, outputFile);
fprintf('\n============== MIL: 11 сценариев ==============\n');
disp(results);
fprintf('Таблица: %s\n', outputFile);
fprintf('Успешных/граничных: %d из %d\n', ...
    sum(Status=="OK" | Status=="BOUNDARY_INF" | Status=="BOUNDARY_NAN"), n);

if AppliedSelect(end) ~= RequestedSelect(end)
    fprintf(['ВНИМАНИЕ: SelectSensor имеет тип int8; запрос 255 ' ...
        'был преобразован в %d. Это тест default для недопустимого ' ...
        'int8-значения, но НЕ буквальный тест 255.\n'], AppliedSelect(end));
end
fprintf('Сделайте скриншот таблицы в Command Window и сохраните CSV.\n');
