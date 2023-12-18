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

```sh
cd ~/study/distance_fw/spresense/sdk
scp kato@43.30.157.109:/home/kato/work/cxm150x-spresense/spresense/sdk/nuttx.spk .;./tools/flash.sh -c COM20 nuttx.spk
```

```
tools/mkcmd.py eltres_calc_distance "ELTRES calc distance application"
make distclean;tools/config.py examples/eltres_calc_distance;make;ls -l nuttx.spk
make;ls -l nuttx.spk
```

#define EEPROM_P1_POW_MODE_OFFSET (0x0450)
#define EEPROM_P2_POW_MODE_OFFSET (0x0650)

```
eltres_eeprom get 0x0450
eltres_eeprom put 0x0450 0x00000000
eltres_eeprom get 0x0650
eltres_eeprom put 0x0650 0x00000000
```



# 残件/課題
- 実機で見えている課題
  - NMEAが出てくるまで時間がかかる
    - fetching-time中に出てこない。
    - 間欠測位動作になってる。→EEPROMの値の書き換え→work/cxm150x-spresense/spresense/examples/eltres_eeprom でいけるはず。

アドレス:0x0650
GNSS 動作モード設定 ※1 
0：常時測位
1：位置優先間欠測位 
2：電力優先間欠測位
3 : 起動時のみ測位 ※2
```
eltres_eeprom get 0x0650
```

- コード構成
  - 複数のAppで参照されるライブラリコードを1箇所で管理したい
    - external下に置きたい。
  - config系#defineの構成方法知りたい
  - spresenseリポジトリ下ではなく、別のディレクトリでリポジトリ管理したい。

- 距離計算の検証
  - web-pageにある既知の値との比較(単体検証)
  - ランダム生成された位置情報からの比較検証
  - 実行結果jsonからリアルタイムに誤差を検証

- Payload生成
  - 送信2点間+乖離値方式
  - 仮数、指数方式