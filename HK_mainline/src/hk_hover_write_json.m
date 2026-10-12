function hk_hover_write_json(file,value)
fid=fopen(file,'w','n','UTF-8');assert(fid>=0,'hk:file','无法写入 %s',file);
c=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
