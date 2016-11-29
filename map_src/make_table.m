
function table = make_table (compatibility, H),

table = zeros(size(compatibility.ic));
i = find(H);
if nnz(i)
    j = H(i);
    i = sub2ind(size(table), i, j);
    table(i) = 1;
end
table = table(:, compatibility.candidates.features);



