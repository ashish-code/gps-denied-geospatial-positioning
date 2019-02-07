import osm_objects
tObj = osm_objects.Objects(True)
tObj.loadFromFile('../../data/00.bin')


for t in subObjs:
    if t.isPoint():
#        print t,t.type,t.kind,t.sha,t.getPoint()
        None
    elif t.isPolyline() and t.type == 'street':
#        print t.tag
        if 'name' in t.tag:
            print 'name = {0}'.format(t.tag['name'])
        if 'oneway' in t.tag:
            print 'oneway = {0}'.format(t.tag['oneway'])
#        for a in t.tag:
#            print a
##        print t,t.type,t.kind,t.sha,t.getPolyline()
#        tPoly = t.getPolyline()
##        print tPoly.points[0].sha,tPoly.points[-1].sha
#        for (n,tRef) in enumerate(tPoly.points):
#            print '{0}: sha: {1} ptr: {2}'.format(n,tRef.sha,tRef.ptr)
#    elif t.isRelation():
#        print t,t.type,t.kind,t.sha,t.getRelation()
#    elif t.isReference():
#        print t,t.type,t.kind,t.sha,t.getReference()
#    else:
#        print 'Unknown tObject type', t,t.type,t.kind,t.sha

