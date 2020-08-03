#ifndef COM_DEF__12_10_2018__08_51
#define COM_DEF__12_10_2018__08_51

#define SDIR(v)         (v<<0)                     /*!< USIC_CH SCTR: SDIR (Bit 0)                                  */
#define PDL(v)          (v<<1)                     /*!< USIC_CH SCTR: PDL (Bit 1)                                   */
#define DSM(v)          (v<<2)                     /*!< USIC_CH SCTR: DSM (Bit 2)                                   */
#define HPCDIR(v)       (v<<4)                     /*!< USIC_CH SCTR: HPCDIR (Bit 4)                                */
#define DOCFG(v)        (v<<6)                     /*!< USIC_CH SCTR: DOCFG (Bit 6)                                 */
#define TRM(v)          (v<<8)                     /*!< USIC_CH SCTR: TRM (Bit 8)                                   */
#define FLE(v)          (v<<16)                    /*!< USIC_CH SCTR: FLE (Bit 16)                                  */
#define WLE(v)          (v<<24)                    /*!< USIC_CH SCTR: WLE (Bit 24)                                  */

#define CLKSEL(v)  		(v<<0)                     /*!< USIC_CH BRG: CLKSEL (Bit 0)                                 */
#define TMEN(v)    		(v<<3)                     /*!< USIC_CH BRG: TMEN (Bit 3)                                   */
#define PPPEN(v)   		(v<<4)                     /*!< USIC_CH BRG: PPPEN (Bit 4)                                  */
#define CTQSEL(v)  		(v<<6)                     /*!< USIC_CH BRG: CTQSEL (Bit 6)                                 */
#define PCTQ(v)    		(v<<8)                     /*!< USIC_CH BRG: PCTQ (Bit 8)                                   */
#define DCTQ(v)    		(v<<10)                    /*!< USIC_CH BRG: DCTQ (Bit 10)                                  */
#define PDIV(v)    		(v<<16)                    /*!< USIC_CH BRG: PDIV (Bit 16)                                  */
#define SCLKOSEL(v)		(v<<28)                    /*!< USIC_CH BRG: SCLKOSEL (Bit 28)                              */
#define MCLKCFG(v) 		(v<<29)                    /*!< USIC_CH BRG: MCLKCFG (Bit 29)                               */
#define SCLKCFG(v) 		(v<<30)                    /*!< USIC_CH BRG: SCLKCFG (Bit 30)                               */

/* ---------------------------------  USIC_CH_CCR  -------------------------------- */
#define MODE(v)      	(v<<0UL)                     /*!< USIC_CH CCR: MODE (Bit 0)                                   */
#define HPCEN(v)     	(v<<6UL)                     /*!< USIC_CH CCR: HPCEN (Bit 6)                                  */
#define PM(v)        	(v<<8UL)                     /*!< USIC_CH CCR: PM (Bit 8)                                     */
#define RSIEN     		(1<<10UL)                    /*!< USIC_CH CCR: RSIEN (Bit 10)                                 */
#define DLIEN     		(1<<11UL)                    /*!< USIC_CH CCR: DLIEN (Bit 11)                                 */
#define TSIEN     		(1<<12UL)                    /*!< USIC_CH CCR: TSIEN (Bit 12)                                 */
#define TBIEN     		(1<<13UL)                    /*!< USIC_CH CCR: TBIEN (Bit 13)                                 */
#define RIEN      		(1<<14UL)                    /*!< USIC_CH CCR: RIEN (Bit 14)                                  */
#define AIEN      		(1<<15UL)                    /*!< USIC_CH CCR: AIEN (Bit 15)                                  */
#define BRGIEN    		(1<<16UL)                    /*!< USIC_CH CCR: BRGIEN (Bit 16)                                */

/* -----------------------------  USIC_CH_PCR_ASCMode  ---------------------------- */

#define SMD(v)       	(v<<0UL)                     /*!< USIC_CH PCR_ASCMode: SMD (Bit 0)                            */
#define STPB(v)      	(v<<1UL)                     /*!< USIC_CH PCR_ASCMode: STPB (Bit 1)                           */
#define IDM(v)       	(v<<2UL)                     /*!< USIC_CH PCR_ASCMode: IDM (Bit 2)                            */
#define SBIEN(v)     	(v<<3UL)                     /*!< USIC_CH PCR_ASCMode: SBIEN (Bit 3)                          */
#define CDEN(v)      	(v<<4UL)                     /*!< USIC_CH PCR_ASCMode: CDEN (Bit 4)                           */
#define RNIEN(v)     	(v<<5UL)                     /*!< USIC_CH PCR_ASCMode: RNIEN (Bit 5)                          */
#define FEIEN(v)     	(v<<6UL)                     /*!< USIC_CH PCR_ASCMode: FEIEN (Bit 6)                          */
#define FFIEN(v)     	(v<<7UL)                     /*!< USIC_CH PCR_ASCMode: FFIEN (Bit 7)                          */
#define SP(v)        	(v<<8UL)                     /*!< USIC_CH PCR_ASCMode: SP (Bit 8)                             */
#define PL(v)        	(v<<13UL)                    /*!< USIC_CH PCR_ASCMode: PL (Bit 13)                            */
#define RSTEN(v)     	(v<<16UL)                    /*!< USIC_CH PCR_ASCMode: RSTEN (Bit 16)                         */
#define TSTEN(v)     	(v<<17UL)                    /*!< USIC_CH PCR_ASCMode: TSTEN (Bit 17)                         */
#define MCLK(v)      	(0x80000000UL)                    /*!< USIC_CH PCR_ASCMode: MCLK (Bit 31)                          */

/* --------------------------------  USIC_CH_DX0CR  ------------------------------- */

#define DSEL(v)        	(v<<0UL)                     /*!< USIC_CH DX0CR: DSEL (Bit 0)                                 */
#define INSW(v)        	(v<<4UL)                     /*!< USIC_CH DX0CR: INSW (Bit 4)                                 */
#define DFEN(v)        	(v<<5UL)                     /*!< USIC_CH DX0CR: DFEN (Bit 5)                                 */
#define DSEN(v)        	(v<<6UL)                     /*!< USIC_CH DX0CR: DSEN (Bit 6)                                 */
#define DPOL(v)        	(v<<8UL)                     /*!< USIC_CH DX0CR: DPOL (Bit 8)                                 */
#define SFSEL(v)       	(v<<9UL)                     /*!< USIC_CH DX0CR: SFSEL (Bit 9)                                */
#define CM(v)          	(v<<10UL)                    /*!< USIC_CH DX0CR: CM (Bit 10)                                  */
#define DXS(v)         	(v<<15UL)                    /*!< USIC_CH DX0CR: DXS (Bit 15)                                 */

/* ---------------------------------  USIC_CH_FDR  -------------------------------- */

#define STEP(v)      	(v<<0UL)                     /*!< USIC_CH FDR: STEP (Bit 0)                                   */
#define DM(v)        	(v<<14UL)                    /*!< USIC_CH FDR: DM (Bit 14)                                    */
#define RESULT(v)    	(v<<16UL)                    /*!< USIC_CH FDR: RESULT (Bit 16)                                */

/* --------------------------------  USIC_CH_TCSR  -------------------------------- */

#define WLEMD(v)    	(v<<0UL)                     /*!< USIC_CH TCSR: WLEMD (Bit 0)                                 */
#define SELMD(v)    	(v<<1UL)                     /*!< USIC_CH TCSR: SELMD (Bit 1)                                 */
#define FLEMD(v)    	(v<<2UL)                     /*!< USIC_CH TCSR: FLEMD (Bit 2)                                 */
#define WAMD(v)     	(v<<3UL)                     /*!< USIC_CH TCSR: WAMD (Bit 3)                                  */
#define HPCMD(v)    	(v<<4UL)                     /*!< USIC_CH TCSR: HPCMD (Bit 4)                                 */
#define SOF(v)      	(v<<5UL)                     /*!< USIC_CH TCSR: SOF (Bit 5)                                   */
#define EOF(v)      	(v<<6UL)                     /*!< USIC_CH TCSR: EOF (Bit 6)                                   */
#define TDV      		(1<<7UL)                     /*!< USIC_CH TCSR: TDV (Bit 7)                                   */
#define TDSSM(v)    	(v<<8UL)                     /*!< USIC_CH TCSR: TDSSM (Bit 8)                                 */
#define TDEN(v)     	(v<<10UL)                    /*!< USIC_CH TCSR: TDEN (Bit 10)                                 */
#define TDVTR(v)    	(v<<12UL)                    /*!< USIC_CH TCSR: TDVTR (Bit 12)                                */
#define WA(v)       	(v<<13UL)                    /*!< USIC_CH TCSR: WA (Bit 13)                                   */

/* --------------------------------  USIC_CH_PSCR  -------------------------------- */

#define TXIDLE         	(0x1UL)                   /*!< USIC_CH PSCR: CST0 (Bitfield-Mask: 0x01)                    */
#define RXIDLE         	(0x2UL)                   /*!< USIC_CH PSCR: CST1 (Bitfield-Mask: 0x01)                    */
#define SBD         	(0x4UL)                   /*!< USIC_CH PSCR: CST2 (Bitfield-Mask: 0x01)                    */
#define COL         	(0x8UL)                   /*!< USIC_CH PSCR: CST3 (Bitfield-Mask: 0x01)                    */
#define RNS         	(0x10UL)                  /*!< USIC_CH PSCR: CST4 (Bitfield-Mask: 0x01)                    */
#define FER0         	(0x20UL)                  /*!< USIC_CH PSCR: CST5 (Bitfield-Mask: 0x01)                    */
#define FER1         	(0x40UL)                  /*!< USIC_CH PSCR: CST6 (Bitfield-Mask: 0x01)                    */
#define RFF         	(0x80UL)                  /*!< USIC_CH PSCR: CST7 (Bitfield-Mask: 0x01)                    */
#define TFF         	(0x100UL)                 /*!< USIC_CH PSCR: CST8 (Bitfield-Mask: 0x01)                    */
#define BUSY         	(0x200UL)                 /*!< USIC_CH PSCR: CST9 (Bitfield-Mask: 0x01)                    */
#define RSIF        	(0x400UL)                 /*!< USIC_CH PSCR: CRSIF (Bitfield-Mask: 0x01)                   */
#define DLIF        	(0x800UL)                 /*!< USIC_CH PSCR: CDLIF (Bitfield-Mask: 0x01)                   */
#define TSIF        	(0x1000UL)                /*!< USIC_CH PSCR: CTSIF (Bitfield-Mask: 0x01)                   */
#define TBIF        	(0x2000UL)                /*!< USIC_CH PSCR: CTBIF (Bitfield-Mask: 0x01)                   */
#define RIF         	(0x4000UL)                /*!< USIC_CH PSCR: CRIF (Bitfield-Mask: 0x01)                    */
#define AIF         	(0x8000UL)                /*!< USIC_CH PSCR: CAIF (Bitfield-Mask: 0x01)                    */
#define BRGIF       	(0x10000UL)               /*!< USIC_CH PSCR: CBRGIF (Bitfield-Mask: 0x01)                  */

/* --------------------------------  USIC_CH_KSCFG  ------------------------------- */
#define MODEN           (0x1UL)                   /*!< USIC_CH KSCFG: MODEN (Bitfield-Mask: 0x01)                  */
#define BPMODEN         (0x2UL)                   /*!< USIC_CH KSCFG: BPMODEN (Bitfield-Mask: 0x01)                */
#define NOMCFG(v)		(((v)&3)<<4)                  /*!< USIC_CH KSCFG: NOMCFG (Bitfield-Mask: 0x03)                 */
#define BPNOM           (0x80UL)                  /*!< USIC_CH KSCFG: BPNOM (Bitfield-Mask: 0x01)                  */
#define SUMCFG(v)		(((v)&3)<<8)                 /*!< USIC_CH KSCFG: SUMCFG (Bitfield-Mask: 0x03)                 */
#define BPSUM           (0x800UL)                 /*!< USIC_CH KSCFG: BPSUM (Bitfield-Mask: 0x01)                  */

#endif /*COM_DEF__12_10_2018__08_51*/
