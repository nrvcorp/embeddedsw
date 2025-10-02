#!/usr/bin/env bash
set -euo pipefail

# =====================================================
# setup_xdma_realtime.sh
#   - isolcpus, nohz_full, rcu_nocbs 설정
#   - PCIe‑XDMA IRQ affinity 설정
# =====================================================

### 사용자 설정 ###
# 분리할 코어 번호 (예: 2,3 또는 4-6)
ISO_CORES="2-10"
# IRQ affinity에 사용할 CPU bitmask (예: 코어2만 → 0x4, 코어3만 → 0x8, 2+3 → 0xC)
CPU_MASK_HEX="7FC"
#######################

echo "→ 격리할 코어: $ISO_CORES"
echo "→ IRQ affinity 마스크: 0x$CPU_MASK_HEX"
read -p "계속하시려면 Enter 키를 누르세요…"

# 1) GRUB 설정 업데이트
echo -e "\n>>> 1) /etc/default/grub 에 isolcpus, nohz_full, rcu_nocbs 추가"
GRUB_CFG="/etc/default/grub"
sudo cp "$GRUB_CFG" "${GRUB_CFG}.bak.$(date +%Y%m%d_%H%M%S)"
sudo sed -i -E \
  "s/^(GRUB_CMDLINE_LINUX=\")([^\"]*)(\".*)/\1\2 isolcpus=${ISO_CORES} nohz_full=${ISO_CORES} rcu_nocbs=${ISO_CORES}\3/" \
  "$GRUB_CFG"
echo "-> update-grub 실행"
sudo update-grub

# 2) PCIe‑XDMA IRQ affinity 설정
echo -e "\n>>> 2) PCIe‑XDMA IRQ affinity 설정"
# /proc/interrupts 에서 'xdma' 또는 'PCIe-XDMA' 키워드로 IRQ 번호 찾기
IRQ_LINE=$(grep -i xdma /proc/interrupts | head -n1) || true
if [[ -z "$IRQ_LINE" ]]; then
  echo "!! /proc/interrupts 에서 XDMA IRQ 라인을 찾지 못했습니다."
  echo "   수동으로 확인 후 /proc/irq/<IRQ>/smp_affinity 에 설정하세요."
  exit 1
fi

IRQ_NUM=$(echo "$IRQ_LINE" | awk '{print $1}' | tr -d ':')
echo "→ 찾은 IRQ 번호: $IRQ_NUM"
echo "-> affinity 마스크(0x$CPU_MASK_HEX) 적용"
echo "$CPU_MASK_HEX" | sudo tee /proc/irq/"$IRQ_NUM"/smp_affinity >/dev/null

echo -e "\n✅ 설정 완료!"
echo "  • 재부팅 후 'isolcpus' 설정이 적용되었는지 확인:  cat /proc/cmdline"
echo "  • IRQ affinity 확인:          cat /proc/irq/$IRQ_NUM/smp_affinity"

