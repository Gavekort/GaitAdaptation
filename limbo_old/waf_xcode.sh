(/home/gavekort/GaitAdaptation/limbo_old/waf $1 ) 2> >( sed -E "s|../src/([^/][a-zA-Z/_]+\.cpp)|/home/gavekort/GaitAdaptation/limbo_old/src/\1|g;s|../src/([^/][a-zA-Z/_]+\.hpp)|/home/gavekort/GaitAdaptation/limbo_old/src/\1|g" >&2 )