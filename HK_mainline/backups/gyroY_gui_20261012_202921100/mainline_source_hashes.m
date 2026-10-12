function rows=mainline_source_hashes(root)
% Record the files actually used, including user edits after unpacking.
rows=struct('path',{},'sha256',{});
for folder=[".","src","scripts","models","CAD","assets","third_party"]
 if folder==".",files=dir(fullfile(root,'*.m'));
 else,files=dir(fullfile(root,folder,'**','*'));end
 for k=1:numel(files)
  if files(k).isdir,continue;end
  file=fullfile(string(files(k).folder),string(files(k).name));
  if endsWith(file,[".slxc",".pyc"]),continue;end
  fid=fopen(file,'rb');assert(fid>=0);bytes=fread(fid,inf,'*uint8');fclose(fid);
  md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);
  digest=typecast(md.digest(),'uint8');hash=lower(reshape(dec2hex(digest,2).',1,[]));
  rows(end+1)=struct('path',replace(extractAfter(file,strlength(root)+1),'\','/'),'sha256',string(hash)); %#ok<AGROW>
 end
end
end
