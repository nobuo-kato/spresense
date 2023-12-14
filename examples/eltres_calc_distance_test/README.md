# [ツールインストール/セットアップ](https://developer.sony.com/spresense/development-guides/sdk_set_up_ja.html#_%E9%96%8B%E7%99%BA%E3%83%84%E3%83%BC%E3%83%AB%E3%81%AE%E3%82%BB%E3%83%83%E3%83%88%E3%82%A2%E3%83%83%E3%83%97)
## ツールインストール
git clone --recursive https://github.com/nobuo-kato/spresense.git
wget https://raw.githubusercontent.com/sonydevworld/spresense/master/install-tools.sh
bash install-tools.sh

## セットアップ
source ~/spresenseenv/setup
cd spresense/sdk
tools/config.py examples/eltres_calc_distance_test
make


make;ls -l nuttx.spk

scp kato@43.30.157.109:/home/kato/work/cxm150x-spresense/spresense/sdk/nuttx.spk .;./tools/flash.sh -c COM20 nuttx.spk