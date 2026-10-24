function run_project()
% Execute the four project stages in order and record the actual outcome.
root=fileparts(fileparts(mfilename('fullpath')));
outdir=fullfile(root,'results','simulink');
if ~exist(outdir,'dir'), mkdir(outdir); end
record=struct('status','RUNNING','matlab_version',version,'started',datestr(now,30));
statusFile=fullfile(outdir,'execution_status.json'); write_json(statusFile,record);
diary(fullfile(outdir,'run_log.txt')); cleanup=onCleanup(@() diary('off'));
try
    fprintf('\nStage 1: Dynamic bicycle model\n');
    record.math=test_math();
    model='lateral_bicycle';
    modelFile=fullfile(root,'matlab',[model '.slx']);
    assert(isfile(modelFile),'Missing Simulink model: %s',modelFile);
    load_system(modelFile);
    fprintf('\nStage 2: Steering input response at 20 m/s\n');
    [~,r1]=run_case('step',20,outdir);
    [~,r2]=run_case('sine',20,outdir);
    reports={r1,r2};
    fprintf('\nStage 3: Same steering input at 10 and 30 m/s\n');
    for U=[10 30]
        [~,rep]=run_case('step',U,outdir); reports{end+1}=rep; %#ok<AGROW>
    end
    fprintf('\nStage 4: Lane-shift tracking, offset recovery and steering constraints\n');
    p=parameters();
    for U=p.speeds_mps(:)'
        for kind={'uncontrolled','track'}
            [~,rep]=run_case(kind{1},U,outdir); reports{end+1}=rep; %#ok<AGROW>
        end
    end
    for kind={'offset','limited'}
        [~,rep]=run_case(kind{1},20,outdir); reports{end+1}=rep; %#ok<AGROW>
    end
    % Leave the model ready to inspect in its nominal tracking configuration.
    c=make_case('track',20,p); M=model_matrices(p,20); K=design_controller(M,p);
    configure_model(model,c,M,K); save_system(model); open_system(model);
    record.status='PASS'; record.completed=datestr(now,30); record.cases=reports;
    write_json(statusFile,record);
    fprintf('\nPASS: all four stages and 12 Simulink cases completed.\nCSV output: %s\n',outdir);
catch err
    record.status='FAIL'; record.message=err.message; write_json(statusFile,record);
    rethrow(err);
end
end
