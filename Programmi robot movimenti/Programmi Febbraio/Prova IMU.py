number = input()
if number < 0: 
    goto negative
if number % 2 == 0:
   print "even"
else:
   print "odd"
goto end
label: negative
print "negative"
label: end
print "all done"
