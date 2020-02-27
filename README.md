# location_register
Location Registerは場所を登録することのできるパッケージです.

## BUILD
git clone --recursive https://github.com/rionehome/location_register.git
colcon build --packages-select location_register

## コマンド一覧
REGISTER
- locationsに入っているLocation全てを登録します.

GET
- locationsに何も入っていなければ,登録されているLocation全てを返します.また,locationsに1つでも入っていれば,nameに対応したLocationを返します.
