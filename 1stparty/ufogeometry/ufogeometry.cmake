Include(FetchContent)

FetchContent_Declare(
  ufogeometry
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufogeometry
  GIT_TAG        main
  GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(ufogeometry)