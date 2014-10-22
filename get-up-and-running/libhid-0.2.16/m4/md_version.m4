AC_DEFUN([MD_VERSION_INFO],
  [
    AH_TOP(
#undef MD_VERSION
    )

    MD_MAJOR_VERSION=`echo "AC_PACKAGE_VERSION" | cut -d. -f1`
    MD_MINOR_VERSION=`echo "AC_PACKAGE_VERSION" | cut -d. -f2`
    MD_MICRO_VERSION=`echo "AC_PACKAGE_VERSION" | cut -d. -f3`
    MD_INTERFACE_AGE=`echo "AC_PACKAGE_VERSION" | cut -d. -f4`
    MD_BINARY_AGE=`echo "AC_PACKAGE_VERSION" | cut -d. -f5`
    MD_VERSION=$MD_MAJOR_VERSION.$MD_MINOR_VERSION.$MD_MICRO_VERSION

    AC_DEFINE_UNQUOTED(MD_VERSION, $MD_VERSION)

    AC_SUBST(MD_MAJOR_VERSION)
    AC_SUBST(MD_MINOR_VERSION)
    AC_SUBST(MD_MICRO_VERSION)
    AC_SUBST(MD_INTERFACE_AGE)
    AC_SUBST(MD_BINARY_AGE)
    AC_SUBST(MD_VERSION)

    MD_MICRO_VERSION_NUM=`echo $MD_MICRO_VERSION | sed 's/[[a-zA-Z]]//g'`
  ])