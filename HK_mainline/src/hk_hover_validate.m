function p=hk_hover_validate(p)
% Reject unknown/missing JSON keys and nonfinite/complex/out-of-range inputs.
s=hk_hover_schema();
assert(isstruct(p)&&isscalar(p),'hk:parameters','参数必须是单个结构体。');
assert(isequal(sort(fieldnames(p)),sort({s.key}.')),'hk:parameters','参数字段不完整或含未知字段，请使用当前版本的配置文件。');
for k=1:numel(s)
 v=p.(s(k).key);
 assert((isnumeric(v)||islogical(v))&&isreal(v)&&isscalar(v)&&isfinite(v), ...
  'hk:parameters','%s 必须为有限标量。',s(k).label);
 assert(v>=s(k).min&&v<=s(k).max,'hk:parameters','%s 超出范围 [%g, %g]。',s(k).label,s(k).min,s(k).max);
 if islogical(s(k).default)
  assert(v==0||v==1,'hk:parameters','开关必须是 0 或 1。');p.(s(k).key)=logical(v);
 else,p.(s(k).key)=double(v);end
end
assert(p.seed==floor(p.seed),'hk:parameters','随机种子必须为整数。');
end
