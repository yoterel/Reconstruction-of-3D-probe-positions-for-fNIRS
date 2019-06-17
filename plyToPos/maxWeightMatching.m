% Weighted maximum matching in general graphs.
%
% Originally written by Joris van Rantwijk in Python:
% 
% http://jorisvr.nl/maximummatching.html
%
% Ported to MATLAB,and not optimized, e.g. for modularity, by Daniel R.
% Saunders, 2013 (with permission). BSD license.
% http://danielrsaunders.com. Original header follows:
%
% The algorithm is taken from "Efficient Algorithms for Finding Maximum
% Matching in Graphs" by Zvi Galil, ACM Computing Surveys, 1986.
% It is based on the "blossom" method for finding augmenting paths and
% the "primal-dual" method for finding a matching of maximum weight, both
% due to Jack Edmonds.
% Some ideas came from "Implementation of algorithms for maximum matching
% on non-bipartite graphs" by H.J. Gabow, Standford Ph.D. thesis, 1973.
% 
% A C program for maximum weight matching by Ed Rothberg was used extensively
% to validate this new code.
% """
function outmate = maxWeightMatching(inedges, inmaxcardinality)
% 
%
%
%
%     """Compute a maximum-weighted matching in the general undirected
%     weighted graph given by "edges".  If "maxcardinality" is true,
%     only maximum-cardinality matchings are considered as solutions.
%
%     Edges is a sequence of tuples (i, j, wt) describing an undirected
%     edge between vertex i and vertex j with weight wt.  There is at most
%     one edge between any two vertices; no vertex has an edge to itself.
%     Vertices are identified by consecutive, non-negative integers.
%
%     Return a list "mate", such that mate(i) == j if vertex i is
%     matched to vertex j, and mate(i) == -1 if vertex i is not matched.
%
%     This function takes time O(n ** 3).""

%
% Vertices are numbered 1 .. nvertex.
% Non-trivial blossoms are numbered nvertex+1 .. 2*nvertex
%
% Edges are numbered 1 .. nedge.
% Edge endpoints are numbered 1 .. 2*nedge, such that endpoints
% (2*k-1) and (2*k) both belong to edge k.
%
% Many terms used in the comments (sub-blossom, T-vertex) come from
% the paper by Galil; read the paper before reading this code.
%

global DEBUG  mate nedge edges maxcardinality nvertex endpoint neighbend label labelend inblossom blossomparent blossomchilds blossombase blossomendps bestedge blossombestedges unusedblossoms dualvar queue allowedge;

DEBUG = false;

if ~exist('inmaxcardinality')
    inmaxcardinality = false;
end

% Deal swiftly with empty graphs.
if isempty(inedges)
    outmate = [];
    return;
end
edges = inedges;
maxcardinality = inmaxcardinality;

% Count vertices.
nedge = size(edges, 1);
nvertex = 0;
for k = 1:nedge
    %         assert i >= 0 and j >= 0 and i ~= j
    i = edges(k,1);
    j = edges(k,2);
    if i > nvertex
        nvertex = i;
    end
    if j > nvertex
        nvertex = j;
    end
end

% Find the maximum edge weight.
maxweight = max(edges(:,3));

% If p is an edge endpoint,
% endpoint(p) is the vertex to which endpoint p is attached.
% Not modified by the algorithm.
for p = 1:2*nedge
    endpoint(p) = edges(floor((p+1)/2),mod(p+1,2)+1);
end

% If v is a vertex,
% neighbend(v) is the list of remote endpoints of the edges attached to v.
% Not modified by the algorithm.
neighbend = {};
for i = 1:nvertex
    neighbend{i} = [];
end
for k = 1:size(edges, 1)
    i = edges(k,1);
    j = edges(k,2);
    neighbend{i} = [neighbend{i} 2*k];
    neighbend{j} = [neighbend{j } 2*k-1];
end


% If v is a vertex,
% mate(v) is the remote endpoint of its matched edge, or -1 if it is single
% (i.e. endpoint[mate(v)] is v's partner vertex).
% Initially all vertices are single; updated during augmentation.
mate = -1* ones(1,nvertex);


% If b is a top-level blossom,
% label(b) is 0 if b is unlabeled (free);
%             1 if b is an S-vertex/blossom;
%             2 if b is a T-vertex/blossom.
% The label of a vertex is found by looking at the label of its
% top-level containing blossom.
% If v is a vertex inside a T-blossom,
% label(v) is 2 iff v is reachable from an S-vertex outside the blossom.
% Labels are assigned during a stage and reset after each augmentation.
label = zeros(1,2 * nvertex);

% If b is a labeled top-level blossom,
% labelend(b) is the remote endpoint of the edge through which b obtained
% its label, or -1 if b's base vertex is single.
% If v is a vertex inside a T-blossom and label(v) == 2,
% labelend(v) is the remote endpoint of the edge through which v is
% reachable from outside the blossom.
labelend = -1* ones(1,2 * nvertex);

% If v is a vertex,
% inblossom(v) is the top-level blossom to which v belongs.
% If v is a top-level vertex, v is itself a blossom (a trivial blossom)
% and inblossom(v) == v.
% Initially all vertices are top-level trivial blossoms.
inblossom = 1:nvertex;

% If b is a sub-blossom,
% blossomparent(b) is its immediate parent (sub-)blossom.
% If b is a top-level blossom, blossomparent(b) is -1.
blossomparent = -1*ones(1,2 * nvertex);

% If b is a non-trivial (sub-)blossom,
% blossomchilds{b} is an ordered list of its sub-blossoms, starting with
% the base and going round the blossom.
blossomchilds = {};
for i = 1:(2*nvertex)
    blossomchilds{i} = [];
end

% If b is a (sub-)blossom,
% blossombase(b) is its base VERTEX (i.e. recursive sub-blossom).
blossombase = [1:nvertex -1*ones(1,nvertex)];

% If b is a non-trivial (sub-)blossom,
% blossomendps{b} is a list of endpoints on its connecting edges,
% such that blossomendps{b}(i) is the local endpoint of blossom
% on the edge that connects it to blossomchilds{b}[wrap(i+1)].
blossomendps = {};
for i = 1:(2*nvertex)
    blossomendps{i} = [];
end

% If v is a free vertex (or an unreached vertex inside a T-blossom),
% bestedge(v) is the edge to an S-vertex with least slack,
% or -1 if there is no such edge.
% If b is a (possibly trivial) top-level S-blossom,
% bestedge(b) is the least-slack edge to a different S-blossom,
% or -1 if there is no such edge.
% This is used for efficient computation of delta2 and delta3.
bestedge = -1*ones(1,2 * nvertex);

% If b is a non-trivial top-level S-blossom,
% blossombestedges(b) is a list of least-slack edges to neighbouring
% S-blossoms, or None if no such list has been computed yet.
% This is used for efficient computation of delta3.
blossombestedges = {};
for i = 1:(2*nvertex)
    blossombestedges{i} = [];
end

% List of currently unused blossom numbers.
unusedblossoms = (nvertex+1):(2*nvertex);

% If v is a vertex,
% dualvar(v) = 2 * u(v) where u(v) is the v's variable in the dual
% optimization problem (multiplication by two ensures integer values
% throughout the algorithm if all edge weights are integers).
% If b is a non-trivial blossom,
% dualvar(b) = z(b) where z(b) is b's variable in the dual optimization
% problem.
dualvar = [maxweight*ones(1,nvertex) zeros(1,nvertex)];

% If allowedge(k) is true, edge k has zero slack in the optimization
% problem; if allowedge(k) is false, the edge's slack may or may not
% be zero.
allowedge = zeros(1,nedge);

% Queue of newly discovered S-vertices.
queue = [ ];


% Main loop: continue until no further improvement is possible.
for t = 1:nvertex
    
    % Each iteration of this loop is a "stage".
    % A stage finds an augmenting path and uses that to improve
    % the matching.
    if DEBUG  disp(sprintf('STAGE %d', t-1)); end;
    
    % Remove labels from top-level blossoms/vertices.
    label = zeros(1, 2 * nvertex);
    
    % Forget all about least-slack edges.
    bestedge = -1*ones(1,2 * nvertex);
    for i = (nvertex+1):(2*nvertex)
        blossombestedges{i} = [];
    end
    
    % Loss of labeling means that we can not be sure that currently
    % allowable edges remain allowable througout this stage.
    allowedge = zeros(1,nedge);
    
    % Make queue empty.
    queue = [ ];
    
    % Label single blossoms/vertices with S and put them in the queue.
    for v = 1:nvertex
        if mate(v) == -1 && label(inblossom(v)) == 0
            assignLabel(v, 1, -1);
        end
    end
    
    % Loop until we succeed in augmenting the matching.
    augmented = 0;
    while 1
        
        % Each iteration of this loop is a "substage".
        % A substage tries to find an augmenting path;
        % if found, the path is used to improve the matching and
        % the stage ends. If there is no augmenting path, the
        % primal-dual method is used to pump some slack out of
        % the dual variables.
        if DEBUG disp(sprintf('SUBSTAGE')); end;
        
        % Continue labeling until all vertices which are reachable
        % through an alternating path have got a label.
        while ~isempty(queue) && ~augmented
%                     if DEBUG disp(sprintf('Queue: %s', num2str(queue))); end;
            % Take an S vertex from the queue.
            v = queue(end);
            queue(end) = [];
            if DEBUG disp(sprintf('POP v=%d', v-1)); end; 
            %                 assert label[inblossom(v)] == 1
            
            % Scan its neighbours:
            for p = neighbend{v}
                k = floor((p+1) / 2);
                w = endpoint(p);
                % w is a neighbour to v
                if inblossom(v) == inblossom(w)
                    % this edge is internal to a blossom; ignore it
                    continue;
                end
                if ~allowedge(k)
                    kslack = slack(k);
                    if kslack <= 0
                        % edge k has zero slack => it is allowable
                        allowedge(k) = true;
                    end
                end
                if allowedge(k)
                    if label(inblossom(w)) == 0
                        % (C1) w is a free vertex;
                        % label w with T and label its mate with S (R12).
                        assignLabel(w, 2, otherEnd(p))
                    elseif label(inblossom(w)) == 1
                        % (C2) w is an S-vertex (not in the same blossom);
                        % follow back-links to discover either an
                        % augmenting path or a new blossom.
                        base = scanBlossom(v, w);
                        if base >= 0
                            % Found a new blossom; add it to the blossom
                            % bookkeeping and turn it into an S-blossom.
                            addBlossom(base, k);
                        else
                            % Found an augmenting path; augment the
                            % matching and end this stage.
                            augmentMatching(k);
                            augmented = 1;
                            break
                        end
                    elseif label(w) == 0
                        % w is inside a T-blossom, but w itself has not
                        % yet been reached from outside the blossom;
                        % mark it as reached (we need this to relabel
                        % during T-blossom expansion).
                        %                             assert label[inblossom(w)] == 2
                        label(w) = 2;
                        labelend(w) = otherEnd(p);
                    end
                elseif label(inblossom(w)) == 1
                    % keep track of the least-slack non-allowable edge to
                    % a different S-blossom.
                    b = inblossom(v);
                    if bestedge(b) == -1 || kslack < slack(bestedge(b))
                        bestedge(b) = k;
                    end
                elseif label(w) == 0
                    % w is a free vertex (or an unreached vertex inside
                    % a T-blossom) but we cfan not reach it yet;
                    % keep track of the least-slack edge that reaches w.
                    if bestedge(w) == -1 || kslack < slack(bestedge(w))
                        bestedge(w) = k;
                    end
                end
            end
        end
        
        if augmented
            break
        end
        
        % There is no augmenting path under these constraints;
        % compute delta and reduce slack in the optimization problem.
        % (Note that our vertex dual variables, edge slacks and delta's
        % are pre-multiplied by two.)
        deltatype = -1;
        delta = [];
        deltaedge = [];
        deltablossom = [];
        %
        %             % Verify data structures for delta2/delta3 computation.
        %             if CHECK_DELTA:
        %                 checkDelta2()
        %                 checkDelta3()
        %             end
        % Compute delta1: the minumum value of any vertex dual.
        if ~maxcardinality
            deltatype = 1;
            delta = min(dualvar(1:nvertex));
        end
        
        % Compute delta2: the minimum slack on any edge between
        % an S-vertex and a free vertex.
        for v = 1:nvertex
            if label(inblossom(v)) == 0 && bestedge(v) ~= -1
                d = slack(bestedge(v));
                if deltatype == -1 || d < delta
                    delta = d;
                    deltatype = 2;
                    deltaedge = bestedge(v);
                end
            end
        end
        
        
        % Compute delta3: half the minimum slack on any edge between
        % a pair of S-blossoms.
        for b = 1:(2 * nvertex)
            if ( blossomparent(b) == -1 && label(b) == 1 && bestedge(b) ~= -1)
                kslack = slack(bestedge(b));
                d = kslack / 2;
                %                     if type(kslack) in (int, long):
                %                         assert (kslack % 2) == 0
                %                         d = kslack // 2
                %                     else:
                %                         d = kslack / 2
                %                     end
                if deltatype == -1 || d < delta
                    delta = d;
                    deltatype = 3;
                    deltaedge = bestedge(b);
                end
            end
        end
        
        % Compute delta4: minimum z variable of any T-blossom.
        for b = (nvertex+1):(2*nvertex)
            if ( blossombase(b) >= 0 && blossomparent(b) == -1 && label(b) == 2 && (deltatype == -1 || dualvar(b) < delta) )
                delta = dualvar(b);
                deltatype = 4;
                deltablossom = b;
            end
        end
        
        if deltatype == -1
            % No further improvement possible; max-cardinality optimum
            % reached. Do a final delta update to make the optimum
            % verifyable.
%             assert maxcardinality
            deltatype = 1;
            delta = max(0, min(dualvar(1:nvertex)));
        end
        
        % Update dual variables according to delta.
        for v  = 1:nvertex
            if label(inblossom(v)) == 1
                % S-vertex: 2*u = 2*u - 2*delta
                dualvar(v) = dualvar(v) - delta;
            elseif label(inblossom(v)) == 2
                % T-vertex: 2*u = 2*u + 2*delta
                dualvar(v) = dualvar(v) + delta;
            end
        end
        for b = (nvertex+1):(2*nvertex)
            if blossombase(b) >= 0 && blossomparent(b) == -1
                if label(b) == 1
                    % top-level S-blossom: z = z + 2*delta
                    dualvar(b) = dualvar(b) + delta;
                elseif label(b) == 2
                    % top-level T-blossom: z = z - 2*delta
                    dualvar(b) = dualvar(b) - delta;
                end
            end
        end
        
        % Take action at the point where minimum delta occurred.
        if DEBUG disp(sprintf('delta%d=%f',deltatype, delta)); end;
        if deltatype == 1
            % No further improvement possible; optimum reached.
            break
        elseif deltatype == 2
            % Use the least-slack edge to continue the search.
            allowedge(deltaedge) = true;
            i = edges(deltaedge,1); j = edges(deltaedge,2); wt = edges(deltaedge,3);
            if label(inblossom(i)) == 0
                m = i;
                i = j;
                j = m;
            end
            %                 assert label[inblossom(i)] == 1
            queue = [queue i];
        elseif deltatype == 3
            % Use the least-slack edge to continue the search.
            allowedge(deltaedge) = true;
            i = edges(deltaedge,1); j = edges(deltaedge,2); wt = edges(deltaedge,3);
            %                 assert label[inblossom(i)] == 1
            queue = [queue i];
        elseif deltatype == 4
            % Expand the least-z blossom.
            expandBlossom(deltablossom, false);
        end
        
        % End of a this substage.
    end
    % Stop when no more augmenting path can be found.
    if ~augmented
        break
    end
    
    % End of a stage; expand all S-blossoms which have dualvar = 0.
    for b = (nvertex+1):2*nvertex
        if ( blossomparent(b) == -1 && blossombase(b) >= 0 && label(b) == 1 && dualvar(b) == 0 )
            expandBlossom(b, true);
        end
    end
end
%     % Verify that we reached the optimum solution.
%     if CHECK_OPTIMUM:
%         verifyOptimum()
%     end

% Transform mate[] such that mate(v) is the vertex to which v is paired.
for v = 1:nvertex
    if mate(v) >= 0
        mate(v) = endpoint(mate(v));
    end
end
%     for v in xrange(nvertex):
%         assert mate(v) == -1 or mate[mate(v)] == v
%     end

outmate = mate;

end

function val = pindex(thearray, index)
val = thearray(mod(index,length(thearray))+1);

end

% Return 2 * slack of edge k (does not work inside blossoms).
function theslack = slack(k, edges, dualvar)
global DEBUG edges dualvar;
i = edges(k,1);
j = edges(k,2);
wt = edges(k,3);
theslack = dualvar(i) + dualvar(j) - 2 * wt;

end


% Generate the leaf vertices of a blossom.
function leaves = blossomLeaves(b)
global DEBUG nvertex blossomchilds;

if b <= nvertex
    leaves = b;
else
    leaves = [];
    childList = blossomchilds{b};
    for t = 1:length(childList)
        if childList(t) <= nvertex
            leaves = [leaves childList(t)];
        else
            leafList = blossomLeaves(childList(t));
            for v = 1:length(leafList)
                leaves = [leaves leafList(v)];
            end
        end
    end
end

end

% Assign label t to the top-level blossom containing vertex w
% and record the fact that w was reached through the edge with
% remote endpoint p.
function assignLabel(w, t, p)
global DEBUG inblossom label labelend bestedge queue blossombase endpoint mate;

if DEBUG 
    if p == -1
        disp(sprintf('assignLabel(%d,%d,%d)', w-1, t, -1)); 
    else
        disp(sprintf('assignLabel(%d,%d,%d)', w-1, t, p-1)); 
    end
end

b = inblossom(w);
%         assert label(w) == 0 and label(b) == 0
label(w) = t;
label(b) = t;
labelend(w) = p;
labelend(b) = p;
bestedge(w) = -1;
bestedge(b) = -1;
if t == 1
    % b became an S-vertex/blossom; add it(s vertices) to the queue.
    queue = [queue blossomLeaves(b)];
    
    if DEBUG disp(sprintf('PUSH [%d] ', blossomLeaves(b)-1)); end;                
elseif t == 2
    % b became a T-vertex/blossom; assign label S to its mate.
    % (If b is a non-trivial blossom, its base is the only vertex
    % with an external mate.)
    base = blossombase(b);
    %             assert mate(base) >= 0
    assignLabel(endpoint(mate(base)), 1, otherEnd(mate(base)))
end

end

% Trace back from vertices v and w to discover either a new blossom
% or an augmenting path. Return the base vertex of the new blossom or -1.
function base = scanBlossom(v, w)
global DEBUG label endpoint blossombase inblossom labelend;

if DEBUG disp(sprintf('scanBlossom(%d,%d)',v-1, w-1)); end;


% Trace back from v and w, placing breadcrumbs as we go.
path = [ ];
base = -1;
while v ~= -1 || w ~= -1
    % Look for a breadcrumb in v's blossom or put a new breadcrumb.
    b = inblossom(v);
    if bitand(label(b), 4)
        base = blossombase(b);
        break
    end
    %             assert label(b) == 1
    path = [path b];
    label(b) = 5;
    % Trace one step back.
    %             assert labelend(b) == mate[blossombase(b)]
    if labelend(b) == -1
        % The base of blossom b is single; stop tracing this path.
        v = -1;
    else
        v = endpoint(labelend(b));
        b = inblossom(v);
        %                 assert label(b) == 2
        % b is a T-blossom; trace one more step back.
        %                 assert labelend(b) >= 0
        v = endpoint(labelend(b));
        
    end
    % Swap v and w so that we alternate between both paths.
    if w ~= -1
        m = v;
        v = w;
        w = m;
    end
    % Remove breadcrumbs
end
for b = path
    label(b) = 1;
end
% Return base vertex, if we found one.


end

% Construct a new blossom with given base, containing edge k which
% connects a pair of S vertices. Label the new blossom as S; set its dual
% variable to zero; relabel its T-vertices to S and add them to the queue.
function addBlossom(base, k)
global DEBUG nvertex inblossom unusedblossoms blossombase blossomparent blossomchilds blossomendps label labelend queue neighbend blossombestedges edges endpoint bestedge;
v = edges(k,1);
w = edges(k,2);
wt = edges(k,3);

bb = inblossom(base);
bv = inblossom(v);
bw = inblossom(w);
% Create blossom.
b = unusedblossoms(end);
unusedblossoms(end) = [];

if DEBUG disp(sprintf('addBlossom(%d,%d) (v=%d w=%d) -> %d', base-1, k-1, v-1, w-1, b-1)); end;
            
blossombase(b) = base;
blossomparent(b) = -1;
blossomparent(bb) = b;
% Make list of sub-blossoms and their interconnecting edge endpoints.
blossomchilds{b} = [];
path = [ ];
blossomendps{b} = [];
endps = [ ];
% Trace back from v to base.
while bv ~= bb
    % Add bv to the new blossom.
    blossomparent(bv) = b;
    path = [path bv];
    endps = [endps labelend(bv)];
    %             assert (label(bv) == 2 or
    %                     (label(bv) == 1 and labelend(bv) == mate[blossombase(bv)]))
    % Trace one step back.
    %             assert labelend(bv) >= 0
    v = endpoint(labelend(bv));
    bv = inblossom(v);
end
% Reverse lists, add endpoint that connects the pair of S vertices.
path = [path bb];
path = fliplr(path);
endps = fliplr(endps);
endps = [endps (2*k)-1];
% Trace back from w to base.
while bw ~= bb
    % Add bw to the new blossom.
    blossomparent(bw) = b;
    path = [path bw];
    endps = [endps otherEnd(labelend(bw))];
    %             assert (label(bw) == 2 or
    %                     (label(bw) == 1 and labelend(bw) == mate[blossombase(bw)]))
    %             % Trace one step back.
    %             assert labelend(bw) >= 0
    w = endpoint(labelend(bw));
    bw = inblossom(w);
end
% Set label to S.
%         assert label(bb) == 1
label(b) = 1;
labelend(b) = labelend(bb);
% Set dual variable to zero.
dualvar(b) = 0;
% Relabel vertices.
blossomchilds{b} = path;
leaves = blossomLeaves(b);
for v = leaves
    if label(inblossom(v)) == 2
        % This T-vertex now turns into an S-vertex because it becomes
        % part of an S-blossom; add it to the queue.
        queue = [queue v];
    end
    inblossom(v) = b;
end
% Compute blossombestedges(b).
bestedgeto = -1*ones(1,2 * nvertex);
for bv = path
    if isempty(blossombestedges{bv})
        % This subblossom does not have a list of least-slack edges;
        % get the information from the vertices.
        nblists = [];
        blossomchilds{b} = path;
        leaves = blossomLeaves(bv);
        for v  = leaves
            for p = neighbend{v}
                nblists = [nblists floor((p+1)/2)];
            end
        end
    else
        % Walk this subblossom's least-slack edges.
        nblists = blossombestedges{bv};
    end
    for k = nblists
        i = edges(k,1); j = edges(k,2); wt = edges(k,3);
        if inblossom(j) == b
            m = j;
            j = i;
            i = m;
            
        end
        
        bj = inblossom(j);
        if (bj ~= b && label(bj) == 1 && (bestedgeto(bj) == -1 || slack(k) < slack(bestedgeto(bj))))
            bestedgeto(bj) = k;
        end
    end
    
    % Forget about least-slack edges of the subblossom.
    blossombestedges{bv} = [];
    bestedge(bv) = -1;
end

be = [];
for k = bestedgeto
    if k ~= -1
        be = [be k];
    end
end
blossombestedges{b} = be;

% Select bestedge(b).
bestedge(b) = -1;
for k = blossombestedges{b}
    if bestedge(b) == -1 || slack(k) < slack(bestedge(b))
        bestedge(b) = k;
    end
end

blossomchilds{b} = path;
blossomendps{b} = endps;

if DEBUG disp(sprintf('blossomchilds[%d]=%s', b-1, num2str(blossomchilds{b}-1))); end;

end


% Expand the given top-level blossom.
function expandBlossom(b, endstage)
global DEBUG dualvar label nvertex endpoint  blossomchilds blossomparent blossomendps labelend blossombase bestedge unusedblossoms allowedge inblossom mate;
         
if DEBUG disp(sprintf('expandBlossom(%d,%d) %s', b-1, endstage, num2str(blossomchilds{b}-1))); end;
% Convert sub-blossoms into top-level blossoms.
for s = blossomchilds{b}
    blossomparent(s) = -1;
    if s <= nvertex
        inblossom(s) = s;
    elseif endstage && dualvar(s) == 0
        % Recursively expand this sub-blossom.
        expandBlossom(s, endstage);
    else
        leaves = blossomLeaves(s);
        for v = leaves
            inblossom(v) = s;
        end
    end
end
% If we expand a T-blossom during a stage, its sub-blossoms must be
% relabeled.
if (~endstage) && label(b) == 2
    % Start at the sub-blossom through which the expanding
    % blossom obtained its label, and relabel sub-blossoms untili
    % we reach the base.
    % Figure out through which sub-blossom the expanding blossom
    % obtained its label initially.
    %             assert labelend(b) >= 0
    entrychild = inblossom(endpoint(otherEnd(labelend(b))));
    % Decide in which direction we will go round the blossom.
    j = find(blossomchilds{b} == entrychild)-1;
    if bitand(j, 1)
        % Start index is odd; go forward and wrap.
        j = j - length(blossomchilds{b});
        jstep = 1;
        endptrick = 0;
    else
        % Start index is even; go backward.
        jstep = -1;
        endptrick = 1;
    end
    % Move along the blossom until we get to the base.
    p = labelend(b);
    
    
    %%
    while j ~= 0
        % Relabel the T-sub-blossom.
        label(endpoint(otherEnd(p))) = 0;
        ptemp = pindex(blossomendps{b},j-endptrick);
        ptemp = whichEnd(ptemp,endptrick);
        ptemp = otherEnd(ptemp);
        label(endpoint(ptemp)) = 0;
        assignLabel(endpoint(otherEnd(p)), 2, p);
        % Step to the next S-sub-blossom and note its forward endpoint.
        ptemp = pindex(blossomendps{b},j-endptrick);
        edge = floor((ptemp+1)/2);
        allowedge(edge) = true;
        j = j + jstep;
        p = whichEnd(pindex(blossomendps{b},j-endptrick), endptrick);
        % Step to the next T-sub-blossom.
        allowedge(floor((p+1)/2)) = true;
        j = j + jstep;
    end
    if DEBUG disp(sprintf('label: %s',num2str(label))); end;
    if DEBUG disp(sprintf('allowedge: %s',num2str(allowedge))); end;
    
    %%
    
    % Relabel the base T-sub-blossom WITHOUT stepping through to
    % its mate (so don't call assignLabel).
    bv = pindex(blossomchilds{b},j);
    label(bv) = 2;
    label(endpoint(otherEnd(p))) = label(bv);
    labelend(bv) = p;
    labelend(endpoint(otherEnd(p))) = labelend(bv);
    bestedge(bv) = -1;
    % Continue along the blossom until we get back to entrychild.
    if DEBUG disp(sprintf('label: %s',num2str(label))); end;
    if DEBUG disp(sprintf('allowedge: %s',num2str(allowedge))); end;
    j = j + jstep;
    while pindex(blossomchilds{b},j) ~= entrychild
        % Examine the vertices of the sub-blossom to see whether
        % it is reachable from a neighbouring S-vertex outside the
        % expanding blossom.
        bv = pindex(blossomchilds{b},j);
        if label(bv) == 1
            % This sub-blossom just got label S through one of its
            % neighbours; leave it.
            j = j + jstep;
            continue
        end
        leaves = blossomLeaves(bv);
        for v = leaves
            if label(v) ~= 0
                break
            end
        end
        % If the sub-blossom contains a reachable vertex, assign
        % label T to the sub-blossom.
        if label(v) ~= 0
            %                     assert label(v) == 2
            %                     assert inblossom(v) == bv
            label(v) = 0;
            label(endpoint(mate(blossombase(bv)))) = 0;
            assignLabel(v, 2, labelend(v));
        end
        j = j + jstep;
    end
    if DEBUG disp(sprintf('label: %s',num2str(label))); end;
    if DEBUG disp(sprintf('allowedge: %s',num2str(allowedge))); end;
end
% Recycle the blossom number.
labelend(b) = -1;
label(b) = -1;
blossomchilds{b} = {};
blossomendps{b} = {};
blossombase(b) = -1;
blossombestedges{b} = {};
bestedge(b) = -1;
unusedblossoms = [unusedblossoms b];
end

% Swap matched/unmatched edges over an alternating path through blossom b
% between vertex v and the base vertex. Keep blossom bookkeeping consistent.
function augmentBlossom(b, v)
global DEBUG blossomparent nvertex blossomchilds blossomendps blossombase endpoint mate;

if DEBUG disp(sprintf('blossomchilds{b}: %s',num2str(blossomchilds{b}-1))); end;
if DEBUG disp(sprintf('blossomendps{b}: %s',num2str(blossomendps{b}-1))); end;
if DEBUG disp(sprintf('blossombase: %s',num2str(blossombase-1))); end;
if DEBUG disp(sprintf('mate: %s',num2str(mate-1))); end;

if DEBUG disp(sprintf('augmentBlossom(%d,%d)', b-1, v-1)); end;
% Bubble up through the blossom tree from vertex v to an immediate
% sub-blossom of b.
t = v;
while blossomparent(t) ~= b
    t = blossomparent(t);
end
% Recursively deal with the first sub-blossom.
if t > nvertex
    augmentBlossom(t, v);
end
% Decide in which direction we will go round the blossom.
i = find(blossomchilds{b}==t)-1;
j = i;
if bitand(i,1)
    % Start index is odd; go forward and wrap.
    j = j - length(blossomchilds{b});
    jstep = 1;
    endptrick = 0;
else
    % Start index is even; go backward.
    jstep = -1;
    endptrick = 1;
end
% Move along the blossom until we get to the base.
while j ~= 0
    % Step to the next sub-blossom and augment it recursively.
    j = j + jstep;
    t = pindex(blossomchilds{b},j);
    p = whichEnd(pindex(blossomendps{b},j-endptrick), endptrick);
    if t > nvertex
        augmentBlossom(t, endpoint(p));
    end
    % Step to the next sub-blossom and augment it recursively.
    j = j + jstep;
    t = pindex(blossomchilds{b},j);
    if t > nvertex
        augmentBlossom(t, endpoint(otherEnd(p)));
    end
    % Match the edge connecting those sub-blossoms.
    mate(endpoint(p)) = otherEnd(p);
    mate(endpoint(otherEnd(p))) = p;
    
    if DEBUG disp(sprintf('PAIR %d %d (k=%d)', endpoint(p), endpoint(otherEnd(p)), floor((p+1)/2))); end;
end
% Rotate the list of sub-blossoms to put the new base at the front.
blossomchilds{b} = [blossomchilds{b}((i+1):end),  blossomchilds{b}(1:i)];
blossomendps{b}  = [blossomendps{b}((i+1):end),  blossomendps{b}(1:i)];
blossombase(b) = blossombase(blossomchilds{b}(1));



%         assert blossombase(b) == v
end

% Swap matched/unmatched edges over an alternating path between two
% single vertices. The augmenting path runs through edge k, which
% connects a pair of S vertices.
function augmentMatching(k)
global DEBUG edges inblossom endpoint labelend mate nvertex;
v = edges(k,1); w = edges(k,2); wt = edges(k,3);
if DEBUG disp(sprintf('augmentMatching(%d) (v=%d w=%d)', k-1, v-1, w-1)); end;
if DEBUG disp(sprintf('PAIR %d %d (k=%d)', v-1, w-1, k-1)); end;
%         for (s, p) in ((v, 2*k+1), (w, 2*k)):
% if DEBUG disp(sprintf('Mate %s', num2str(mate))); end;
% if DEBUG disp(sprintf('Labelend %s', num2str(labelend))); end;

for i = 1:2
    if i == 1
        s = v;
        p = 2*k;
    else
        s = w;
        p = 2*k-1;
    end
    % Match vertex s to remote endpoint p. Then trace back from s
    % until we find a single vertex, swapping matched and unmatched
    % edges as we go.
    while 1
        bs = inblossom(s);
        %                 assert label(bs) == 1
        %                 assert labelend(bs) == mate[blossombase(bs)]
        % Augment through the S-blossom from s to base.
        if bs > nvertex
            augmentBlossom(bs, s)
        end
        % Update mate(s)
        mate(s) = p;
        % Trace one step back.
        if labelend(bs) == -1
            % Reached single vertex; stop.
            break
        end
        t = endpoint(labelend(bs));
        bt = inblossom(t);
        %                 assert label(bt) == 2
        % Trace one step back.
        %                 assert labelend(bt) >= 0
        s = endpoint(labelend(bt));
        j = endpoint(otherEnd(labelend(bt)));
        % Augment through the T-blossom from j to base.
        %                 assert blossombase(bt) == t
        if bt > nvertex
            augmentBlossom(bt, j);
        end
        % Update mate(j)
        mate(j) = labelend(bt);
        % Keep the opposite endpoint;
        % it will be assigned to mate(s) in the next step.
        p = otherEnd(labelend(bt));
        if DEBUG disp(sprintf('PAIR %d %d (k=%d)', s-1, t-1, floor((p+1)/2)-1)); end;
    end
end
end


function p = otherEnd(p)
p = bitxor(p-1,1)+1;
end

function p = whichEnd(p,otherOne)
if otherOne
    p = otherEnd(p);
end
end
