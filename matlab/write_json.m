function write_json(path,value)
% Save execution metadata without a third-party JSON dependency.
fid=fopen(path,'w'); assert(fid>=0,'Cannot write %s',path);
cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',jsonencode(value));
end
