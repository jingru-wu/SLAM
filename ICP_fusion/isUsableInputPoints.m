function is_use = isUsableInputPoints(is_close, is_similar, is_first)

    %==== TODO: Compute is_use, indicating input points to add to fusion map based on input booleans ====
    is_use = or(and(is_close,is_similar),is_first);
%     is_use = (is_close & is_similar) | is_first;
end