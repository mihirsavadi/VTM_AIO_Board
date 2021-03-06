<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="6.5.0">
<drawing>
<settings>
<setting alwaysvectorfont="yes"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="4" name="Route4" color="1" fill="4" visible="no" active="no"/>
<layer number="5" name="Route5" color="4" fill="4" visible="no" active="no"/>
<layer number="6" name="Route6" color="1" fill="8" visible="no" active="no"/>
<layer number="7" name="Route7" color="4" fill="8" visible="no" active="no"/>
<layer number="8" name="Route8" color="1" fill="2" visible="no" active="no"/>
<layer number="9" name="Route9" color="4" fill="2" visible="no" active="no"/>
<layer number="10" name="Route10" color="1" fill="7" visible="no" active="no"/>
<layer number="11" name="Route11" color="4" fill="7" visible="no" active="no"/>
<layer number="12" name="Route12" color="1" fill="5" visible="no" active="no"/>
<layer number="13" name="Route13" color="4" fill="5" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="39-30-1240">
<packages>
<package name="39-30-1240">
<wire x1="-25.8" y1="12.8" x2="-25.8" y2="0" width="0.127" layer="21"/>
<wire x1="-25.8" y1="0" x2="25.8" y2="0" width="0.127" layer="21"/>
<wire x1="25.8" y1="0" x2="25.8" y2="12.8" width="0.127" layer="21"/>
<wire x1="25.8" y1="12.8" x2="-25.8" y2="12.8" width="0.127" layer="21"/>
<pad name="P$1" x="23.1" y="13.9" drill="1.8" shape="square"/>
<hole x="23.1" y="6.6" drill="3"/>
<hole x="-23.1" y="6.6" drill="3"/>
<pad name="P$2" x="18.9" y="13.9" drill="1.8"/>
<pad name="P$3" x="14.7" y="13.9" drill="1.8"/>
<pad name="P$4" x="10.5" y="13.9" drill="1.8"/>
<pad name="P$5" x="6.3" y="13.9" drill="1.8"/>
<pad name="P$6" x="2.1" y="13.9" drill="1.8"/>
<pad name="P$7" x="-2.1" y="13.9" drill="1.8"/>
<pad name="P$8" x="-6.3" y="13.9" drill="1.8"/>
<pad name="P$9" x="-10.5" y="13.9" drill="1.8"/>
<pad name="P$10" x="-14.7" y="13.9" drill="1.8"/>
<pad name="P$11" x="-18.9" y="13.9" drill="1.8"/>
<pad name="P$12" x="-23.1" y="13.9" drill="1.8"/>
<pad name="P$13" x="23.1" y="19.4" drill="1.8"/>
<pad name="P$14" x="18.9" y="19.4" drill="1.8"/>
<pad name="P$15" x="14.7" y="19.4" drill="1.8"/>
<pad name="P$16" x="10.5" y="19.4" drill="1.8"/>
<pad name="P$17" x="6.3" y="19.4" drill="1.8"/>
<pad name="P$18" x="2.1" y="19.4" drill="1.8"/>
<pad name="P$19" x="-2.1" y="19.4" drill="1.8"/>
<pad name="P$20" x="-6.3" y="19.4" drill="1.8"/>
<pad name="P$21" x="-10.5" y="19.4" drill="1.8"/>
<pad name="P$22" x="-14.7" y="19.4" drill="1.8"/>
<pad name="P$23" x="-18.9" y="19.4" drill="1.8"/>
<pad name="P$24" x="-23.1" y="19.4" drill="1.8"/>
<text x="-15" y="9" size="1.27" layer="21">&gt;NAME</text>
<text x="-15" y="5" size="1.27" layer="21">&gt;VALUE</text>
<text x="25.4" y="13.97" size="1.27" layer="21">1</text>
</package>
</packages>
<symbols>
<symbol name="39-30-1240">
<wire x1="-33.02" y1="7.62" x2="-33.02" y2="-7.62" width="0.254" layer="94"/>
<wire x1="-33.02" y1="-7.62" x2="30.48" y2="-7.62" width="0.254" layer="94"/>
<wire x1="30.48" y1="-7.62" x2="30.48" y2="7.62" width="0.254" layer="94"/>
<wire x1="30.48" y1="7.62" x2="-33.02" y2="7.62" width="0.254" layer="94"/>
<text x="-25.4" y="-12.7" size="1.27" layer="94">&gt;NAME</text>
<text x="-25.4" y="-15.24" size="1.27" layer="94">&gt;VALUE</text>
<pin name="P$1" x="27.94" y="10.16" length="short" rot="R270"/>
<pin name="P$2" x="22.86" y="10.16" length="short" rot="R270"/>
<pin name="P$3" x="17.78" y="10.16" length="short" rot="R270"/>
<pin name="P$4" x="12.7" y="10.16" length="short" rot="R270"/>
<pin name="P$5" x="7.62" y="10.16" length="short" rot="R270"/>
<pin name="P$6" x="2.54" y="10.16" length="short" rot="R270"/>
<pin name="P$7" x="-2.54" y="10.16" length="short" rot="R270"/>
<pin name="P$8" x="-7.62" y="10.16" length="short" rot="R270"/>
<pin name="P$9" x="-12.7" y="10.16" length="short" rot="R270"/>
<pin name="P$10" x="-17.78" y="10.16" length="short" rot="R270"/>
<pin name="P$11" x="-22.86" y="10.16" length="short" rot="R270"/>
<pin name="P$12" x="-27.94" y="10.16" length="short" rot="R270"/>
<pin name="P$13" x="25.4" y="15.24" rot="R270"/>
<pin name="P$14" x="20.32" y="15.24" rot="R270"/>
<pin name="P$15" x="15.24" y="15.24" rot="R270"/>
<pin name="P$16" x="10.16" y="15.24" rot="R270"/>
<pin name="P$17" x="5.08" y="15.24" rot="R270"/>
<pin name="P$18" x="0" y="15.24" rot="R270"/>
<pin name="P$19" x="-5.08" y="15.24" rot="R270"/>
<pin name="P$20" x="-10.16" y="15.24" rot="R270"/>
<pin name="P$21" x="-15.24" y="15.24" rot="R270"/>
<pin name="P$22" x="-20.32" y="15.24" rot="R270"/>
<pin name="P$23" x="-25.4" y="15.24" rot="R270"/>
<pin name="P$24" x="-30.48" y="15.24" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="39-30-1240">
<gates>
<gate name="G$1" symbol="39-30-1240" x="0" y="0"/>
</gates>
<devices>
<device name="" package="39-30-1240">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$10" pad="P$10"/>
<connect gate="G$1" pin="P$11" pad="P$11"/>
<connect gate="G$1" pin="P$12" pad="P$12"/>
<connect gate="G$1" pin="P$13" pad="P$13"/>
<connect gate="G$1" pin="P$14" pad="P$14"/>
<connect gate="G$1" pin="P$15" pad="P$15"/>
<connect gate="G$1" pin="P$16" pad="P$16"/>
<connect gate="G$1" pin="P$17" pad="P$17"/>
<connect gate="G$1" pin="P$18" pad="P$18"/>
<connect gate="G$1" pin="P$19" pad="P$19"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
<connect gate="G$1" pin="P$20" pad="P$20"/>
<connect gate="G$1" pin="P$21" pad="P$21"/>
<connect gate="G$1" pin="P$22" pad="P$22"/>
<connect gate="G$1" pin="P$23" pad="P$23"/>
<connect gate="G$1" pin="P$24" pad="P$24"/>
<connect gate="G$1" pin="P$3" pad="P$3"/>
<connect gate="G$1" pin="P$4" pad="P$4"/>
<connect gate="G$1" pin="P$5" pad="P$5"/>
<connect gate="G$1" pin="P$6" pad="P$6"/>
<connect gate="G$1" pin="P$7" pad="P$7"/>
<connect gate="G$1" pin="P$8" pad="P$8"/>
<connect gate="G$1" pin="P$9" pad="P$9"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="CRCW_2010_XXX_X_JNEF">
<packages>
<package name="CRCW_2010_390R_JNEF">
<smd name="P$1" x="2.55" y="0" dx="1.2" dy="2.5" layer="1"/>
<smd name="P$2" x="-2.55" y="0" dx="1.2" dy="2.5" layer="1"/>
<wire x1="-2.5" y1="1.25" x2="2.5" y2="1.25" width="0.127" layer="21"/>
<wire x1="2.5" y1="1.25" x2="2.5" y2="-1.25" width="0.127" layer="21"/>
<wire x1="2.5" y1="-1.25" x2="-2.5" y2="-1.25" width="0.127" layer="21"/>
<wire x1="-2.5" y1="-1.25" x2="-2.5" y2="1.25" width="0.127" layer="21"/>
<text x="-3" y="2" size="1.27" layer="21">&gt;NAME</text>
<text x="-3" y="-3" size="1.27" layer="21">&gt;VALUE</text>
</package>
<package name="CRCW_2010_10K0_JNEF">
<smd name="P$2" x="-2.55" y="0" dx="2.5" dy="1.2" layer="1" rot="R90"/>
<smd name="P$1" x="2.55" y="0" dx="2.5" dy="1.2" layer="1" rot="R90"/>
<wire x1="-2.5" y1="1.25" x2="2.5" y2="1.25" width="0.127" layer="21"/>
<wire x1="2.5" y1="1.25" x2="2.5" y2="-1.25" width="0.127" layer="21"/>
<wire x1="2.5" y1="-1.25" x2="-2.5" y2="-1.25" width="0.127" layer="21"/>
<wire x1="-2.5" y1="-1.25" x2="-2.5" y2="1.25" width="0.127" layer="21"/>
<text x="-3" y="3" size="1.27" layer="21">&gt;NAME</text>
<text x="-3" y="-4" size="1.27" layer="21">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="CRCW_2010_390R_JNEF">
<wire x1="-7.62" y1="2.54" x2="7.62" y2="2.54" width="0.254" layer="94"/>
<wire x1="7.62" y1="2.54" x2="7.62" y2="-2.54" width="0.254" layer="94"/>
<wire x1="7.62" y1="-2.54" x2="-7.62" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-2.54" x2="-7.62" y2="2.54" width="0.254" layer="94"/>
<pin name="P$1" x="10.16" y="0" length="short" rot="R180"/>
<pin name="P$2" x="-10.16" y="0" length="short"/>
<text x="-2.54" y="5.08" size="1.778" layer="94">&gt;NAME</text>
<text x="-2.54" y="-7.62" size="1.778" layer="94">&gt;VALUE</text>
</symbol>
<symbol name="CRCW_2010_10K0_JNEF">
<wire x1="-7.62" y1="2.54" x2="7.62" y2="2.54" width="0.254" layer="94"/>
<wire x1="7.62" y1="2.54" x2="7.62" y2="-2.54" width="0.254" layer="94"/>
<wire x1="7.62" y1="-2.54" x2="-7.62" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-2.54" x2="-7.62" y2="2.54" width="0.254" layer="94"/>
<pin name="P$1" x="10.16" y="0" length="short" rot="R180"/>
<pin name="P$2" x="-10.16" y="0" length="short"/>
<text x="-2.54" y="5.08" size="1.778" layer="94">&gt;NAME</text>
<text x="-2.54" y="-7.62" size="1.778" layer="94">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="CRCW_2010_390R_JNEF">
<gates>
<gate name="G$1" symbol="CRCW_2010_390R_JNEF" x="0" y="0"/>
</gates>
<devices>
<device name="" package="CRCW_2010_390R_JNEF">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="CRCW_2010_10K0_JNEF">
<gates>
<gate name="G$1" symbol="CRCW_2010_10K0_JNEF" x="0" y="0"/>
</gates>
<devices>
<device name="" package="CRCW_2010_10K0_JNEF">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="Dialight_By_element14_Batch_1">
<description>Developed by element14 :&lt;br&gt;
element14 CAD Library consolidation.ulp
at 13/06/2012 13:59:22</description>
<packages>
<package name="LED_597-3111-407F">
<smd name="1" x="-1.4986" y="0" dx="1.397" dy="1.6002" layer="1"/>
<smd name="2" x="1.4986" y="0" dx="1.397" dy="1.6002" layer="1"/>
<wire x1="3.4036" y1="-0.0508" x2="2.6416" y2="-0.0508" width="0.1524" layer="21"/>
<wire x1="3.0226" y1="0.3302" x2="3.0226" y2="-0.4318" width="0.1524" layer="21"/>
<wire x1="-0.4572" y1="-0.762" x2="0.4572" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="0.4572" y1="0.762" x2="-0.4572" y2="0.762" width="0.1524" layer="21"/>
<wire x1="3.4036" y1="-0.0508" x2="2.6416" y2="-0.0508" width="0.1524" layer="51"/>
<wire x1="3.0226" y1="0.3302" x2="3.0226" y2="-0.4318" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="-0.762" x2="1.4986" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="-0.762" x2="1.4986" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="0.762" x2="-1.4986" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="0.762" x2="-1.4986" y2="-0.762" width="0.1524" layer="51"/>
<text x="-4.699" y="1.397" size="2.0828" layer="25" ratio="10" rot="SR0">&gt;NAME</text>
<text x="-5.715" y="-3.3528" size="2.0828" layer="27" ratio="10" rot="SR0">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="597-3111-407F">
<pin name="2" x="0" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
<pin name="1" x="10.16" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<wire x1="3.81" y1="1.905" x2="3.81" y2="0" width="0.2032" layer="94"/>
<wire x1="3.81" y1="0" x2="3.81" y2="-1.905" width="0.2032" layer="94"/>
<wire x1="2.54" y1="0" x2="3.81" y2="0" width="0.2032" layer="94"/>
<wire x1="6.35" y1="-1.905" x2="6.35" y2="0" width="0.2032" layer="94"/>
<wire x1="6.35" y1="0" x2="6.35" y2="1.905" width="0.2032" layer="94"/>
<wire x1="6.35" y1="0" x2="7.62" y2="0" width="0.2032" layer="94"/>
<wire x1="6.35" y1="4.445" x2="6.985" y2="3.81" width="0.2032" layer="94"/>
<wire x1="6.985" y1="3.81" x2="8.255" y2="5.08" width="0.2032" layer="94"/>
<wire x1="8.255" y1="3.81" x2="8.89" y2="3.175" width="0.2032" layer="94"/>
<wire x1="8.89" y1="3.175" x2="10.16" y2="4.445" width="0.2032" layer="94"/>
<wire x1="8.255" y1="5.08" x2="7.62" y2="5.08" width="0.2032" layer="94"/>
<wire x1="7.62" y1="5.08" x2="8.255" y2="4.445" width="0.2032" layer="94"/>
<wire x1="8.255" y1="4.445" x2="8.255" y2="5.08" width="0.2032" layer="94"/>
<wire x1="10.16" y1="4.445" x2="9.525" y2="4.445" width="0.2032" layer="94"/>
<wire x1="9.525" y1="4.445" x2="10.16" y2="3.81" width="0.2032" layer="94"/>
<wire x1="10.16" y1="3.81" x2="10.16" y2="4.445" width="0.2032" layer="94"/>
<wire x1="6.985" y1="2.54" x2="8.255" y2="3.81" width="0.2032" layer="94"/>
<wire x1="6.35" y1="0" x2="3.81" y2="1.905" width="0.2032" layer="94"/>
<wire x1="3.81" y1="-1.905" x2="6.35" y2="0" width="0.2032" layer="94"/>
<wire x1="5.08" y1="3.175" x2="6.35" y2="4.445" width="0.2032" layer="94"/>
<text x="-4.3942" y="-9.3472" size="3.4798" layer="96" ratio="10" rot="SR0">&gt;VALUE</text>
<text x="-3.4036" y="3.302" size="2.0828" layer="95" ratio="10" rot="SR0">&gt;NAME</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="597-3111-407F" prefix="LED">
<description>LED 1206 ALGAAS RED SMT</description>
<gates>
<gate name="A" symbol="597-3111-407F" x="0" y="0"/>
</gates>
<devices>
<device name="" package="LED_597-3111-407F">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COLOR" value="AlGaAs Red" constant="no"/>
<attribute name="MPN" value="597-3111-407F" constant="no"/>
<attribute name="OC_FARNELL" value="1519490" constant="no"/>
<attribute name="OC_NEWARK" value="30K4900" constant="no"/>
<attribute name="PACKAGE" value="1206-RED LED" constant="no"/>
<attribute name="SUPPLIER" value="Dialight" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="Molex 39-30-1160">
<packages>
<package name="39-30-1160">
<wire x1="-17.4" y1="6.4" x2="17.4" y2="6.4" width="0.127" layer="21"/>
<wire x1="17.4" y1="6.4" x2="17.4" y2="-6.4" width="0.127" layer="21"/>
<wire x1="17.4" y1="-6.4" x2="-17.4" y2="-6.4" width="0.127" layer="21"/>
<wire x1="-17.4" y1="-6.4" x2="-17.4" y2="6.4" width="0.127" layer="21"/>
<hole x="-14.7" y="0.2" drill="3"/>
<hole x="14.7" y="0.2" drill="3"/>
<pad name="P$1" x="14.7" y="7.5" drill="1.8" shape="square"/>
<pad name="P$2" x="10.5" y="7.5" drill="1.8"/>
<pad name="P$3" x="6.3" y="7.5" drill="1.8"/>
<pad name="P$4" x="2.1" y="7.5" drill="1.8"/>
<pad name="P$5" x="-2.1" y="7.5" drill="1.8"/>
<pad name="P$6" x="-6.3" y="7.5" drill="1.8"/>
<pad name="P$7" x="-10.5" y="7.5" drill="1.8"/>
<pad name="P$8" x="-14.7" y="7.5" drill="1.8"/>
<pad name="P$9" x="14.7" y="13" drill="1.8"/>
<pad name="P$10" x="10.5" y="13" drill="1.8"/>
<pad name="P$11" x="6.3" y="13" drill="1.8"/>
<pad name="P$12" x="2.1" y="13" drill="1.8"/>
<pad name="P$13" x="-2.1" y="13" drill="1.8"/>
<pad name="P$14" x="-6.3" y="13" drill="1.8"/>
<pad name="P$15" x="-10.5" y="13" drill="1.8"/>
<pad name="P$16" x="-14.7" y="13" drill="1.8"/>
<text x="-6.67" y="3.46" size="1.27" layer="21">&gt;NAME</text>
<text x="-6.67" y="-3.35" size="1.27" layer="21">&gt;VALUE</text>
<text x="16.51" y="7.62" size="1.27" layer="21">1</text>
</package>
</packages>
<symbols>
<symbol name="39-30-1160">
<wire x1="-22.86" y1="5.08" x2="22.86" y2="5.08" width="0.254" layer="94"/>
<wire x1="22.86" y1="5.08" x2="22.86" y2="-5.08" width="0.254" layer="94"/>
<wire x1="22.86" y1="-5.08" x2="-22.86" y2="-5.08" width="0.254" layer="94"/>
<wire x1="-22.86" y1="-5.08" x2="-22.86" y2="5.08" width="0.254" layer="94"/>
<pin name="P$1" x="20.32" y="10.16" length="middle" rot="R270"/>
<pin name="P$2" x="15.24" y="10.16" length="middle" rot="R270"/>
<pin name="P$3" x="10.16" y="10.16" length="middle" rot="R270"/>
<pin name="P$4" x="5.08" y="10.16" length="middle" rot="R270"/>
<pin name="P$5" x="-2.54" y="10.16" length="middle" rot="R270"/>
<pin name="P$6" x="-7.62" y="10.16" length="middle" rot="R270"/>
<pin name="P$7" x="-12.7" y="10.16" length="middle" rot="R270"/>
<pin name="P$8" x="-17.78" y="10.16" length="middle" rot="R270"/>
<pin name="P$9" x="17.78" y="12.7" rot="R270"/>
<pin name="P$10" x="12.7" y="12.7" rot="R270"/>
<pin name="P$11" x="7.62" y="12.7" rot="R270"/>
<pin name="P$12" x="2.54" y="12.7" rot="R270"/>
<pin name="P$13" x="-5.08" y="12.7" rot="R270"/>
<pin name="P$14" x="-10.16" y="12.7" rot="R270"/>
<pin name="P$15" x="-15.24" y="12.7" rot="R270"/>
<pin name="P$16" x="-20.32" y="12.7" rot="R270"/>
<text x="25.4" y="5.08" size="1.27" layer="94">&gt;NAME</text>
<text x="25.4" y="2.54" size="1.27" layer="94">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="39-30-1160">
<gates>
<gate name="G$1" symbol="39-30-1160" x="0" y="0"/>
</gates>
<devices>
<device name="" package="39-30-1160">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$10" pad="P$10"/>
<connect gate="G$1" pin="P$11" pad="P$11"/>
<connect gate="G$1" pin="P$12" pad="P$12"/>
<connect gate="G$1" pin="P$13" pad="P$13"/>
<connect gate="G$1" pin="P$14" pad="P$14"/>
<connect gate="G$1" pin="P$15" pad="P$15"/>
<connect gate="G$1" pin="P$16" pad="P$16"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
<connect gate="G$1" pin="P$3" pad="P$3"/>
<connect gate="G$1" pin="P$4" pad="P$4"/>
<connect gate="G$1" pin="P$5" pad="P$5"/>
<connect gate="G$1" pin="P$6" pad="P$6"/>
<connect gate="G$1" pin="P$7" pad="P$7"/>
<connect gate="G$1" pin="P$8" pad="P$8"/>
<connect gate="G$1" pin="P$9" pad="P$9"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="17861650002">
<packages>
<package name="178.6165.0002">
<wire x1="0" y1="0" x2="20" y2="0" width="0.127" layer="21"/>
<wire x1="20" y1="0" x2="20" y2="6" width="0.127" layer="21"/>
<wire x1="20" y1="6" x2="0" y2="6" width="0.127" layer="21"/>
<wire x1="0" y1="6" x2="0" y2="0" width="0.127" layer="21"/>
<hole x="10" y="3" drill="2.4"/>
<pad name="P5" x="7.1" y="1.75" drill="1.4"/>
<pad name="P8" x="3.6" y="1.75" drill="1.4"/>
<pad name="P7" x="3.6" y="4.25" drill="1.4"/>
<pad name="P6" x="7.1" y="4.25" drill="1.4"/>
<pad name="P3" x="12.9" y="4.25" drill="1.4"/>
<pad name="P2" x="16.4" y="4.25" drill="1.4"/>
<pad name="P1" x="16.4" y="1.75" drill="1.4"/>
<pad name="P4" x="12.9" y="1.75" drill="1.4"/>
</package>
</packages>
<symbols>
<symbol name="178.6165.0002">
<wire x1="0" y1="0" x2="0" y2="12.7" width="0.254" layer="94"/>
<wire x1="0" y1="12.7" x2="33.02" y2="12.7" width="0.254" layer="94"/>
<wire x1="33.02" y1="12.7" x2="33.02" y2="0" width="0.254" layer="94"/>
<wire x1="33.02" y1="0" x2="0" y2="0" width="0.254" layer="94"/>
<pin name="P1" x="5.08" y="-5.08" length="middle" rot="R90"/>
<pin name="P2" x="12.7" y="-5.08" length="middle" rot="R90"/>
<pin name="P3" x="20.32" y="-5.08" length="middle" rot="R90"/>
<pin name="P4" x="27.94" y="-5.08" length="middle" rot="R90"/>
<pin name="P8" x="5.08" y="17.78" length="middle" rot="R270"/>
<pin name="P7" x="12.7" y="17.78" length="middle" rot="R270"/>
<pin name="P6" x="20.32" y="17.78" length="middle" rot="R270"/>
<pin name="P5" x="27.94" y="17.78" length="middle" rot="R270"/>
<text x="12.7" y="5.08" size="1.27" layer="94">FUSE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="178.6165.0002">
<gates>
<gate name="G$1" symbol="178.6165.0002" x="0" y="0"/>
</gates>
<devices>
<device name="" package="178.6165.0002">
<connects>
<connect gate="G$1" pin="P1" pad="P1"/>
<connect gate="G$1" pin="P2" pad="P2"/>
<connect gate="G$1" pin="P3" pad="P3"/>
<connect gate="G$1" pin="P4" pad="P4"/>
<connect gate="G$1" pin="P5" pad="P5"/>
<connect gate="G$1" pin="P6" pad="P6"/>
<connect gate="G$1" pin="P7" pad="P7"/>
<connect gate="G$1" pin="P8" pad="P8"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="DDZ20C-7">
<packages>
<package name="DDZ20C-7">
<wire x1="-1.35" y1="0.775" x2="1.35" y2="0.775" width="0.127" layer="21"/>
<wire x1="1.35" y1="0.775" x2="1.35" y2="-0.775" width="0.127" layer="21"/>
<wire x1="1.35" y1="-0.775" x2="-1.35" y2="-0.775" width="0.127" layer="21"/>
<wire x1="-1.35" y1="-0.775" x2="-1.35" y2="0.775" width="0.127" layer="21"/>
<smd name="P$1" x="1.575" y="0" dx="0.95" dy="0.9" layer="1" rot="R90"/>
<smd name="P$2" x="-1.575" y="0" dx="0.95" dy="0.9" layer="1" rot="R90"/>
<wire x1="-0.75" y1="0.5" x2="-0.75" y2="-0.5" width="0.127" layer="21"/>
<text x="-3" y="2" size="1.27" layer="21">&gt;NAME</text>
<text x="-3" y="-3" size="1.27" layer="21">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="DDZ20C-7">
<wire x1="-2.54" y1="0" x2="5.08" y2="5.08" width="0.254" layer="94"/>
<wire x1="5.08" y1="5.08" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<wire x1="5.08" y1="-5.08" x2="-2.54" y2="0" width="0.254" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="-2.54" y2="-5.08" width="0.254" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="0" y2="7.62" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-5.08" x2="-5.08" y2="-7.62" width="0.254" layer="94"/>
<pin name="P$1" x="10.16" y="0" length="middle" rot="R180"/>
<pin name="P$2" x="-7.62" y="0" length="middle"/>
<text x="2.54" y="10.16" size="1.778" layer="94">&gt;NAME</text>
<text x="2.54" y="-10.16" size="1.778" layer="94">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="DDZ20C-7">
<gates>
<gate name="G$1" symbol="DDZ20C-7" x="-2.54" y="0"/>
</gates>
<devices>
<device name="" package="DDZ20C-7">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="42820-4223">
<packages>
<package name="42820-4223">
<wire x1="-21.91" y1="0" x2="-21.91" y2="21" width="0.127" layer="21"/>
<wire x1="-21.91" y1="21" x2="21.91" y2="21" width="0.127" layer="21"/>
<wire x1="21.91" y1="21" x2="21.91" y2="0" width="0.127" layer="21"/>
<wire x1="21.91" y1="0" x2="-21.91" y2="0" width="0.127" layer="21"/>
<hole x="20.45" y="13.56" drill="3.05"/>
<hole x="-20.45" y="13.56" drill="3.05"/>
<pad name="P$1" x="14.72" y="22.56" drill="2.85" shape="square"/>
<pad name="P$2" x="4.72" y="22.56" drill="2.85"/>
<pad name="P$3" x="-4.72" y="22.56" drill="2.85"/>
<pad name="P$4" x="-14.72" y="22.56" drill="2.85"/>
<pad name="P$5" x="14.72" y="27.56" drill="2.85" shape="square"/>
<pad name="P$6" x="4.72" y="27.56" drill="2.85"/>
<pad name="P$7" x="-4.72" y="27.56" drill="2.85"/>
<pad name="P$8" x="-14.72" y="27.56" drill="2.85"/>
<text x="-10" y="15" size="1.27" layer="21">&gt;NAME</text>
<text x="-11" y="8" size="1.27" layer="21">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="42820-4223">
<wire x1="-20.32" y1="0" x2="22.86" y2="0" width="0.254" layer="94"/>
<wire x1="22.86" y1="0" x2="22.86" y2="15.24" width="0.254" layer="94"/>
<wire x1="22.86" y1="15.24" x2="-20.32" y2="15.24" width="0.254" layer="94"/>
<wire x1="-20.32" y1="15.24" x2="-20.32" y2="0" width="0.254" layer="94"/>
<pin name="P$1" x="17.78" y="17.78" length="short" rot="R270"/>
<pin name="P$2" x="10.16" y="17.78" length="short" rot="R270"/>
<pin name="P$3" x="-5.08" y="17.78" length="short" rot="R270"/>
<pin name="P$4" x="-15.24" y="17.78" length="short" rot="R270"/>
<pin name="P$5" x="15.24" y="22.86" rot="R270"/>
<pin name="P$6" x="7.62" y="22.86" rot="R270"/>
<pin name="P$7" x="-7.62" y="22.86" rot="R270"/>
<pin name="P$8" x="-17.78" y="22.86" rot="R270"/>
<text x="-15.24" y="5.08" size="1.27" layer="94">&gt;NAME</text>
<text x="-15.24" y="2.54" size="1.27" layer="94">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="42820-4223">
<gates>
<gate name="G$1" symbol="42820-4223" x="0" y="0"/>
</gates>
<devices>
<device name="" package="42820-4223">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
<connect gate="G$1" pin="P$3" pad="P$3"/>
<connect gate="G$1" pin="P$4" pad="P$4"/>
<connect gate="G$1" pin="P$5" pad="P$5"/>
<connect gate="G$1" pin="P$6" pad="P$6"/>
<connect gate="G$1" pin="P$7" pad="P$7"/>
<connect gate="G$1" pin="P$8" pad="P$8"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="ER3JB-TP">
<packages>
<package name="ER3JB-TP">
<smd name="P$1" x="-2.05" y="0" dx="2.159" dy="2.159" layer="1"/>
<smd name="P$2" x="2.05" y="0" dx="2.159" dy="2.159" layer="1" rot="R180"/>
<wire x1="-2.3495" y1="1.9685" x2="2.3495" y2="1.9685" width="0.127" layer="21"/>
<wire x1="2.3495" y1="1.9685" x2="2.3495" y2="-1.9685" width="0.127" layer="21"/>
<wire x1="2.3495" y1="-1.9685" x2="-2.3495" y2="-1.9685" width="0.127" layer="21"/>
<wire x1="-2.3495" y1="-1.9685" x2="-2.3495" y2="1.9685" width="0.127" layer="21"/>
<wire x1="0.508" y1="1.27" x2="0.508" y2="0" width="0.127" layer="21"/>
<wire x1="0.508" y1="0" x2="0.508" y2="-1.27" width="0.127" layer="21"/>
<wire x1="0.508" y1="0" x2="-0.508" y2="1.27" width="0.127" layer="21"/>
<wire x1="-0.508" y1="1.27" x2="-0.508" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-0.508" y1="-1.27" x2="0.508" y2="0" width="0.127" layer="21"/>
<text x="-2.54" y="2.54" size="1.27" layer="21">&gt;NAME</text>
<text x="-2.54" y="-3.81" size="1.27" layer="21">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="ER3JB-TP">
<wire x1="0" y1="2.54" x2="0" y2="-2.54" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="5.08" y2="0" width="0.254" layer="94"/>
<wire x1="5.08" y1="0" x2="0" y2="2.54" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="0" width="0.254" layer="94"/>
<wire x1="5.08" y1="0" x2="5.08" y2="-2.54" width="0.254" layer="94"/>
<pin name="P$1" x="-5.08" y="0" length="middle"/>
<pin name="P$2" x="10.16" y="0" length="middle" rot="R180"/>
<text x="0" y="5.08" size="1.27" layer="94">&gt;NAME</text>
<text x="0" y="-5.08" size="1.27" layer="94">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="ER3JB-TP">
<gates>
<gate name="G$1" symbol="ER3JB-TP" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ER3JB-TP">
<connects>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="transistor-fet">
<description>&lt;b&gt;Field Effect Transistors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;&lt;p&gt;
&lt;p&gt;
Symbols changed according to IEC617&lt;p&gt; 
All types, packages and assignment to symbols and pins checked&lt;p&gt;
Package outlines partly checked&lt;p&gt;
&lt;p&gt;
JFET = junction FET&lt;p&gt;
IGBT-x = insulated gate bipolar transistor&lt;p&gt;
x=N: NPN; x=P: PNP&lt;p&gt;
IGFET-mc-nnn; (IGFET=insulated gate field effect transistor)&lt;P&gt;
m=D: depletion mode (Verdr&amp;auml;ngungstyp)&lt;p&gt;
m=E: enhancement mode (Anreicherungstyp)&lt;p&gt;
c: N=N-channel; P=P-Channel&lt;p&gt;
GDSB: gate, drain, source, bulk&lt;p&gt;
&lt;p&gt;
by R. Vogg  15.March.2002</description>
<packages>
<package name="TO220">
<description>&lt;b&gt;TO 220 horizontal&lt;/b&gt;</description>
<wire x1="-5.207" y1="-1.27" x2="5.207" y2="-1.27" width="0.127" layer="21"/>
<wire x1="5.207" y1="14.605" x2="-5.207" y2="14.605" width="0.127" layer="21"/>
<wire x1="5.207" y1="-1.27" x2="5.207" y2="11.176" width="0.127" layer="21"/>
<wire x1="5.207" y1="11.176" x2="4.318" y2="11.176" width="0.127" layer="21"/>
<wire x1="4.318" y1="11.176" x2="4.318" y2="12.7" width="0.127" layer="21"/>
<wire x1="4.318" y1="12.7" x2="5.207" y2="12.7" width="0.127" layer="21"/>
<wire x1="5.207" y1="12.7" x2="5.207" y2="14.605" width="0.127" layer="21"/>
<wire x1="-5.207" y1="-1.27" x2="-5.207" y2="11.176" width="0.127" layer="21"/>
<wire x1="-5.207" y1="11.176" x2="-4.318" y2="11.176" width="0.127" layer="21"/>
<wire x1="-4.318" y1="11.176" x2="-4.318" y2="12.7" width="0.127" layer="21"/>
<wire x1="-4.318" y1="12.7" x2="-5.207" y2="12.7" width="0.127" layer="21"/>
<wire x1="-5.207" y1="12.7" x2="-5.207" y2="14.605" width="0.127" layer="21"/>
<wire x1="-4.572" y1="-0.635" x2="4.572" y2="-0.635" width="0.0508" layer="21"/>
<wire x1="4.572" y1="7.62" x2="4.572" y2="-0.635" width="0.0508" layer="21"/>
<wire x1="4.572" y1="7.62" x2="-4.572" y2="7.62" width="0.0508" layer="21"/>
<wire x1="-4.572" y1="-0.635" x2="-4.572" y2="7.62" width="0.0508" layer="21"/>
<circle x="0" y="11.176" radius="1.8034" width="0.127" layer="21"/>
<circle x="0" y="11.176" radius="4.191" width="0" layer="42"/>
<circle x="0" y="11.176" radius="4.191" width="0" layer="43"/>
<pad name="1" x="-2.54" y="-6.35" drill="1.1176" shape="long" rot="R90"/>
<pad name="2" x="0" y="-6.35" drill="1.1176" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="-6.35" drill="1.1176" shape="long" rot="R90"/>
<text x="-3.81" y="5.207" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.937" y="2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
<text x="-4.445" y="7.874" size="0.9906" layer="21" ratio="12">A17,5mm</text>
<text x="-3.175" y="0" size="1.27" layer="51" ratio="10">1</text>
<text x="-0.635" y="0" size="1.27" layer="51" ratio="10">2</text>
<text x="1.905" y="0" size="1.27" layer="51" ratio="10">3</text>
<rectangle x1="2.159" y1="-4.699" x2="2.921" y2="-4.064" layer="21"/>
<rectangle x1="-0.381" y1="-4.699" x2="0.381" y2="-4.064" layer="21"/>
<rectangle x1="-2.921" y1="-4.699" x2="-2.159" y2="-4.064" layer="21"/>
<rectangle x1="-3.175" y1="-4.064" x2="-1.905" y2="-1.27" layer="21"/>
<rectangle x1="-0.635" y1="-4.064" x2="0.635" y2="-1.27" layer="21"/>
<rectangle x1="1.905" y1="-4.064" x2="3.175" y2="-1.27" layer="21"/>
<rectangle x1="-2.921" y1="-6.604" x2="-2.159" y2="-4.699" layer="51"/>
<rectangle x1="-0.381" y1="-6.604" x2="0.381" y2="-4.699" layer="51"/>
<rectangle x1="2.159" y1="-6.604" x2="2.921" y2="-4.699" layer="51"/>
<hole x="0" y="11.176" drill="3.302"/>
</package>
</packages>
<symbols>
<symbol name="IGFET-EP-GDS">
<wire x1="-2.54" y1="-2.54" x2="-1.2192" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="0" y1="0.762" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="0" y2="-0.762" width="0.254" layer="94"/>
<wire x1="0" y1="3.683" x2="0" y2="1.397" width="0.254" layer="94"/>
<wire x1="0.635" y1="0.635" x2="1.905" y2="0" width="0.254" layer="94"/>
<wire x1="0.635" y1="-0.635" x2="1.905" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="1.905" y2="0" width="0.1524" layer="94"/>
<wire x1="1.905" y1="0" x2="2.54" y2="0" width="0.1524" layer="94"/>
<wire x1="2.54" y1="0" x2="2.54" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="0" y1="-1.397" x2="0" y2="-3.683" width="0.254" layer="94"/>
<wire x1="-1.143" y1="2.54" x2="-1.143" y2="-2.54" width="0.254" layer="94"/>
<text x="-11.43" y="0" size="1.778" layer="96">&gt;VALUE</text>
<text x="-11.43" y="2.54" size="1.778" layer="95">&gt;NAME</text>
<pin name="D" x="5.08" y="2.54" visible="off" length="middle" direction="pas" rot="R180"/>
<pin name="S" x="5.08" y="-2.54" visible="off" length="middle" direction="pas" rot="R180"/>
<pin name="G" x="-5.08" y="-2.54" visible="off" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="BUZ172" prefix="Q">
<description>&lt;b&gt;P-Channel Enhancement MOSFET&lt;/b&gt; -100V; -5A; 0,8Ohm</description>
<gates>
<gate name="G$1" symbol="IGFET-EP-GDS" x="0" y="0"/>
</gates>
<devices>
<device name="" package="TO220">
<connects>
<connect gate="G$1" pin="D" pad="2"/>
<connect gate="G$1" pin="G" pad="1"/>
<connect gate="G$1" pin="S" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
<class number="1" name="Signal" width="0.4064" drill="0.254">
<clearance class="1" value="0.254"/>
</class>
<class number="2" name="Power" width="1.4224" drill="1.5494">
<clearance class="2" value="0.254"/>
</class>
<class number="3" name="Drivers" width="1.4224" drill="1.5494">
<clearance class="3" value="0.254"/>
</class>
</classes>
<parts>
<part name="POWER_BUS" library="39-30-1240" deviceset="39-30-1240" device="" value="J3"/>
<part name="PDU" library="Dialight_By_element14_Batch_1" deviceset="597-3111-407F" device="" value="PWR_LED"/>
<part name="LED_R" library="CRCW_2010_XXX_X_JNEF" deviceset="CRCW_2010_390R_JNEF" device="" value="500"/>
<part name="U$1" library="17861650002" deviceset="178.6165.0002" device=""/>
<part name="U$2" library="17861650002" deviceset="178.6165.0002" device=""/>
<part name="ZDIODE" library="DDZ20C-7" deviceset="DDZ20C-7" device="" value=""/>
<part name="POWER" library="42820-4223" deviceset="42820-4223" device="" value="J1"/>
<part name="FP&amp;IGN_D1" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="FP&amp;IGN_R" library="CRCW_2010_XXX_X_JNEF" deviceset="CRCW_2010_10K0_JNEF" device="" value="10k"/>
<part name="FP&amp;IGN_D2" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="FP&amp;IGN" library="transistor-fet" deviceset="BUZ172" device="" value="P085T"/>
<part name="UPS_R" library="CRCW_2010_XXX_X_JNEF" deviceset="CRCW_2010_10K0_JNEF" device="" value="10k"/>
<part name="UPS_D1" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="UPS_D2" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="UPS" library="transistor-fet" deviceset="BUZ172" device="" value="P085T"/>
<part name="DNS_R" library="CRCW_2010_XXX_X_JNEF" deviceset="CRCW_2010_10K0_JNEF" device="" value="10k"/>
<part name="DNS_D1" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="DNS_D2" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="DNS" library="transistor-fet" deviceset="BUZ172" device="" value="P085T"/>
<part name="CLTCH_R" library="CRCW_2010_XXX_X_JNEF" deviceset="CRCW_2010_10K0_JNEF" device="" value="10k"/>
<part name="CLTCH_D1" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="CLTCH_D2" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="CLTCH" library="transistor-fet" deviceset="BUZ172" device="" value="P085T"/>
<part name="CLTCH-H_R" library="CRCW_2010_XXX_X_JNEF" deviceset="CRCW_2010_10K0_JNEF" device="" value="10k"/>
<part name="CLTCH-H_D1" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="CLTCH-H_D2" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="CLTCH-H" library="transistor-fet" deviceset="BUZ172" device="" value="P085T"/>
<part name="BL_R" library="CRCW_2010_XXX_X_JNEF" deviceset="CRCW_2010_10K0_JNEF" device="" value="10k"/>
<part name="BL_D1" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="BL_D2" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="BL" library="transistor-fet" deviceset="BUZ172" device="" value="P085T"/>
<part name="FAN_R" library="CRCW_2010_XXX_X_JNEF" deviceset="CRCW_2010_10K0_JNEF" device="" value="10k"/>
<part name="FAN_D1" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="FAN_D2" library="ER3JB-TP" deviceset="ER3JB-TP" device="" value=""/>
<part name="FAN" library="transistor-fet" deviceset="BUZ172" device="" value="P085T"/>
<part name="I/O" library="Molex 39-30-1160" deviceset="39-30-1160" device="" value="J2"/>
</parts>
<sheets>
<sheet>
<plain>
<text x="-66.04" y="22.86" size="1.778" layer="91">J4</text>
<text x="-144.78" y="0" size="1.778" layer="91">INDICATOR LED</text>
<text x="58.42" y="-12.7" size="1.778" layer="91">J2</text>
<text x="55.88" y="43.18" size="1.778" layer="91">J3</text>
<text x="76.2" y="58.42" size="1.778" layer="91">MOTEC 20A FUSE</text>
<text x="73.66" y="-27.94" size="1.778" layer="91">FUEL PUMP 20A FUSE</text>
</plain>
<instances>
<instance part="POWER_BUS" gate="G$1" x="-68.58" y="25.4" rot="R270"/>
<instance part="PDU" gate="A" x="-137.16" y="12.7" rot="R90"/>
<instance part="LED_R" gate="G$1" x="-137.16" y="35.56" rot="R90"/>
<instance part="U$1" gate="G$1" x="91.44" y="20.32" rot="R90"/>
<instance part="U$2" gate="G$1" x="91.44" y="-22.86" rot="R90"/>
<instance part="ZDIODE" gate="G$1" x="-134.62" y="-38.1" rot="R270"/>
<instance part="POWER" gate="G$1" x="-73.66" y="-48.26" rot="R270"/>
<instance part="I/O" gate="G$1" x="40.64" y="30.48" rot="R90"/>
</instances>
<busses>
</busses>
<nets>
<net name="+12VS" class="2">
<segment>
<pinref part="POWER_BUS" gate="G$1" pin="P$10"/>
<wire x1="-58.42" y1="43.18" x2="-38.1" y2="43.18" width="0.1524" layer="91"/>
<wire x1="-38.1" y1="43.18" x2="-38.1" y2="48.26" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$11"/>
<wire x1="-58.42" y1="48.26" x2="-38.1" y2="48.26" width="0.1524" layer="91"/>
<wire x1="-38.1" y1="43.18" x2="-25.4" y2="43.18" width="0.1524" layer="91"/>
<junction x="-38.1" y="43.18"/>
<label x="-27.94" y="43.18" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U$1" gate="G$1" pin="P8"/>
<wire x1="73.66" y1="25.4" x2="71.12" y2="25.4" width="0.1524" layer="91"/>
<wire x1="71.12" y1="25.4" x2="71.12" y2="33.02" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P7"/>
<wire x1="71.12" y1="33.02" x2="73.66" y2="33.02" width="0.1524" layer="91"/>
<wire x1="71.12" y1="33.02" x2="71.12" y2="40.64" width="0.1524" layer="91"/>
<junction x="71.12" y="33.02"/>
<pinref part="U$1" gate="G$1" pin="P6"/>
<wire x1="71.12" y1="40.64" x2="73.66" y2="40.64" width="0.1524" layer="91"/>
<wire x1="71.12" y1="40.64" x2="71.12" y2="48.26" width="0.1524" layer="91"/>
<junction x="71.12" y="40.64"/>
<pinref part="U$1" gate="G$1" pin="P5"/>
<wire x1="71.12" y1="48.26" x2="73.66" y2="48.26" width="0.1524" layer="91"/>
<label x="68.58" y="33.02" size="1.778" layer="95" rot="R90"/>
</segment>
<segment>
<pinref part="ZDIODE" gate="G$1" pin="P$2"/>
<wire x1="-134.62" y1="-30.48" x2="-134.62" y2="-25.4" width="0.1524" layer="91"/>
<label x="-132.08" y="-25.4" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="POWER" gate="G$1" pin="P$1"/>
<wire x1="-55.88" y1="-66.04" x2="-40.64" y2="-66.04" width="0.1524" layer="91"/>
<wire x1="-40.64" y1="-66.04" x2="-40.64" y2="-63.5" width="0.1524" layer="91"/>
<pinref part="POWER" gate="G$1" pin="P$5"/>
<wire x1="-40.64" y1="-63.5" x2="-50.8" y2="-63.5" width="0.1524" layer="91"/>
<label x="-35.56" y="-66.04" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="PDU" gate="A" pin="2"/>
<wire x1="-137.16" y1="12.7" x2="-137.16" y2="5.08" width="0.1524" layer="91"/>
<label x="-137.16" y="5.08" size="1.778" layer="95"/>
</segment>
</net>
<net name="GND" class="2">
<segment>
<pinref part="POWER_BUS" gate="G$1" pin="P$1"/>
<wire x1="-58.42" y1="-2.54" x2="-38.1" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="-38.1" y1="-2.54" x2="-38.1" y2="2.54" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$3"/>
<wire x1="-38.1" y1="2.54" x2="-38.1" y2="7.62" width="0.1524" layer="91"/>
<wire x1="-38.1" y1="7.62" x2="-58.42" y2="7.62" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$2"/>
<wire x1="-58.42" y1="2.54" x2="-38.1" y2="2.54" width="0.1524" layer="91"/>
<junction x="-38.1" y="2.54"/>
<wire x1="-38.1" y1="-2.54" x2="-25.4" y2="-2.54" width="0.1524" layer="91"/>
<junction x="-38.1" y="-2.54"/>
<label x="-27.94" y="-2.54" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="LED_R" gate="G$1" pin="P$1"/>
<wire x1="-137.16" y1="45.72" x2="-137.16" y2="55.88" width="0.1524" layer="91"/>
<label x="-142.24" y="53.34" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="ZDIODE" gate="G$1" pin="P$1"/>
<wire x1="-134.62" y1="-48.26" x2="-134.62" y2="-53.34" width="0.1524" layer="91"/>
<label x="-134.62" y="-53.34" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="POWER" gate="G$1" pin="P$2"/>
<wire x1="-55.88" y1="-58.42" x2="-40.64" y2="-58.42" width="0.1524" layer="91"/>
<wire x1="-40.64" y1="-58.42" x2="-40.64" y2="-55.88" width="0.1524" layer="91"/>
<pinref part="POWER" gate="G$1" pin="P$6"/>
<wire x1="-40.64" y1="-55.88" x2="-50.8" y2="-55.88" width="0.1524" layer="91"/>
<label x="-35.56" y="-58.42" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="POWER" gate="G$1" pin="P$3"/>
<wire x1="-55.88" y1="-43.18" x2="-40.64" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="-40.64" y1="-43.18" x2="-40.64" y2="-40.64" width="0.1524" layer="91"/>
<pinref part="POWER" gate="G$1" pin="P$7"/>
<wire x1="-40.64" y1="-40.64" x2="-50.8" y2="-40.64" width="0.1524" layer="91"/>
<label x="-35.56" y="-43.18" size="1.778" layer="95"/>
</segment>
</net>
<net name="0VE_BUS" class="2">
<segment>
<pinref part="POWER_BUS" gate="G$1" pin="P$13"/>
<wire x1="-53.34" y1="0" x2="-33.02" y2="0" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="0" x2="-33.02" y2="5.08" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$15"/>
<wire x1="-33.02" y1="5.08" x2="-33.02" y2="10.16" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="10.16" x2="-53.34" y2="10.16" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$14"/>
<wire x1="-53.34" y1="5.08" x2="-33.02" y2="5.08" width="0.1524" layer="91"/>
<junction x="-33.02" y="5.08"/>
<wire x1="-33.02" y1="0" x2="-25.4" y2="0" width="0.1524" layer="91"/>
<junction x="-33.02" y="0"/>
<label x="-27.94" y="0" size="1.778" layer="95"/>
</segment>
</net>
<net name="0VA_BUS" class="2">
<segment>
<pinref part="POWER_BUS" gate="G$1" pin="P$17"/>
<wire x1="-53.34" y1="20.32" x2="-33.02" y2="20.32" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="20.32" x2="-33.02" y2="17.78" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$4"/>
<wire x1="-33.02" y1="17.78" x2="-33.02" y2="15.24" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="15.24" x2="-33.02" y2="12.7" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="12.7" x2="-58.42" y2="12.7" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$16"/>
<wire x1="-53.34" y1="15.24" x2="-33.02" y2="15.24" width="0.1524" layer="91"/>
<junction x="-33.02" y="15.24"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$5"/>
<wire x1="-58.42" y1="17.78" x2="-33.02" y2="17.78" width="0.1524" layer="91"/>
<junction x="-33.02" y="17.78"/>
<wire x1="-33.02" y1="12.7" x2="-25.4" y2="12.7" width="0.1524" layer="91"/>
<junction x="-33.02" y="12.7"/>
<label x="-27.94" y="12.7" size="1.778" layer="95"/>
</segment>
</net>
<net name="5VA_BUS" class="2">
<segment>
<pinref part="POWER_BUS" gate="G$1" pin="P$6"/>
<wire x1="-58.42" y1="22.86" x2="-38.1" y2="22.86" width="0.1524" layer="91"/>
<wire x1="-38.1" y1="22.86" x2="-38.1" y2="27.94" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$9"/>
<wire x1="-38.1" y1="27.94" x2="-38.1" y2="33.02" width="0.1524" layer="91"/>
<wire x1="-38.1" y1="33.02" x2="-38.1" y2="38.1" width="0.1524" layer="91"/>
<wire x1="-38.1" y1="38.1" x2="-58.42" y2="38.1" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$8"/>
<wire x1="-38.1" y1="33.02" x2="-58.42" y2="33.02" width="0.1524" layer="91"/>
<junction x="-38.1" y="33.02"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$7"/>
<wire x1="-38.1" y1="27.94" x2="-58.42" y2="27.94" width="0.1524" layer="91"/>
<junction x="-38.1" y="27.94"/>
<wire x1="-38.1" y1="22.86" x2="-25.4" y2="22.86" width="0.1524" layer="91"/>
<junction x="-38.1" y="22.86"/>
<label x="-27.94" y="22.86" size="1.778" layer="95"/>
</segment>
</net>
<net name="5VE_BUS" class="2">
<segment>
<pinref part="POWER_BUS" gate="G$1" pin="P$18"/>
<wire x1="-53.34" y1="25.4" x2="-33.02" y2="25.4" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="25.4" x2="-33.02" y2="30.48" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$20"/>
<wire x1="-33.02" y1="30.48" x2="-33.02" y2="35.56" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="35.56" x2="-53.34" y2="35.56" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$19"/>
<wire x1="-53.34" y1="30.48" x2="-33.02" y2="30.48" width="0.1524" layer="91"/>
<junction x="-33.02" y="30.48"/>
<wire x1="-33.02" y1="25.4" x2="-25.4" y2="25.4" width="0.1524" layer="91"/>
<junction x="-33.02" y="25.4"/>
<label x="-27.94" y="25.4" size="1.778" layer="95"/>
</segment>
</net>
<net name="8VE_BUS" class="2">
<segment>
<pinref part="POWER_BUS" gate="G$1" pin="P$21"/>
<wire x1="-53.34" y1="40.64" x2="-33.02" y2="40.64" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="40.64" x2="-33.02" y2="45.72" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$24"/>
<wire x1="-33.02" y1="45.72" x2="-33.02" y2="50.8" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="50.8" x2="-33.02" y2="55.88" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="55.88" x2="-53.34" y2="55.88" width="0.1524" layer="91"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$23"/>
<wire x1="-53.34" y1="50.8" x2="-33.02" y2="50.8" width="0.1524" layer="91"/>
<junction x="-33.02" y="50.8"/>
<pinref part="POWER_BUS" gate="G$1" pin="P$22"/>
<wire x1="-53.34" y1="45.72" x2="-33.02" y2="45.72" width="0.1524" layer="91"/>
<junction x="-33.02" y="45.72"/>
<wire x1="-33.02" y1="40.64" x2="-25.4" y2="40.64" width="0.1524" layer="91"/>
<junction x="-33.02" y="40.64"/>
<label x="-27.94" y="40.64" size="1.778" layer="95"/>
</segment>
</net>
<net name="MOTEC_OUT-F" class="3">
<segment>
<pinref part="POWER_BUS" gate="G$1" pin="P$12"/>
<wire x1="-25.4" y1="53.34" x2="-58.42" y2="53.34" width="0.1524" layer="91"/>
<label x="-27.94" y="53.34" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U$1" gate="G$1" pin="P1"/>
<wire x1="96.52" y1="25.4" x2="99.06" y2="25.4" width="0.1524" layer="91"/>
<wire x1="99.06" y1="25.4" x2="99.06" y2="33.02" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P2"/>
<wire x1="99.06" y1="33.02" x2="96.52" y2="33.02" width="0.1524" layer="91"/>
<wire x1="99.06" y1="33.02" x2="99.06" y2="40.64" width="0.1524" layer="91"/>
<junction x="99.06" y="33.02"/>
<pinref part="U$1" gate="G$1" pin="P3"/>
<wire x1="99.06" y1="40.64" x2="96.52" y2="40.64" width="0.1524" layer="91"/>
<wire x1="99.06" y1="40.64" x2="99.06" y2="48.26" width="0.1524" layer="91"/>
<junction x="99.06" y="40.64"/>
<pinref part="U$1" gate="G$1" pin="P4"/>
<wire x1="99.06" y1="48.26" x2="96.52" y2="48.26" width="0.1524" layer="91"/>
<label x="101.6" y="30.48" size="1.778" layer="95" rot="R90"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="PDU" gate="A" pin="1"/>
<pinref part="LED_R" gate="G$1" pin="P$2"/>
<wire x1="-137.16" y1="22.86" x2="-137.16" y2="25.4" width="0.1524" layer="91"/>
</segment>
</net>
<net name="FAN_OUT" class="2">
<segment>
<pinref part="POWER" gate="G$1" pin="P$4"/>
<wire x1="-55.88" y1="-33.02" x2="-40.64" y2="-33.02" width="0.1524" layer="91"/>
<wire x1="-40.64" y1="-33.02" x2="-40.64" y2="-30.48" width="0.1524" layer="91"/>
<pinref part="POWER" gate="G$1" pin="P$8"/>
<wire x1="-40.64" y1="-30.48" x2="-50.8" y2="-30.48" width="0.1524" layer="91"/>
<label x="-35.56" y="-33.02" size="1.778" layer="95"/>
</segment>
</net>
<net name="DNS_OUT" class="3">
<segment>
<pinref part="I/O" gate="G$1" pin="P$12"/>
<wire x1="27.94" y1="33.02" x2="22.86" y2="33.02" width="0.1524" layer="91"/>
<label x="5.08" y="33.02" size="1.778" layer="95"/>
</segment>
</net>
<net name="BL_OUT" class="3">
<segment>
<pinref part="I/O" gate="G$1" pin="P$13"/>
<wire x1="27.94" y1="25.4" x2="22.86" y2="25.4" width="0.1524" layer="91"/>
<label x="5.08" y="25.4" size="1.778" layer="95"/>
</segment>
</net>
<net name="FP&amp;IGN_OUT" class="3">
<segment>
<pinref part="U$2" gate="G$1" pin="P8"/>
<wire x1="73.66" y1="-17.78" x2="71.12" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="71.12" y1="-17.78" x2="71.12" y2="-10.16" width="0.1524" layer="91"/>
<pinref part="U$2" gate="G$1" pin="P7"/>
<wire x1="71.12" y1="-10.16" x2="73.66" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="71.12" y1="-10.16" x2="71.12" y2="-2.54" width="0.1524" layer="91"/>
<junction x="71.12" y="-10.16"/>
<pinref part="U$2" gate="G$1" pin="P6"/>
<wire x1="71.12" y1="-2.54" x2="73.66" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="71.12" y1="-2.54" x2="71.12" y2="5.08" width="0.1524" layer="91"/>
<junction x="71.12" y="-2.54"/>
<pinref part="U$2" gate="G$1" pin="P5"/>
<wire x1="71.12" y1="5.08" x2="73.66" y2="5.08" width="0.1524" layer="91"/>
<label x="68.58" y="-12.7" size="1.778" layer="95" rot="R90"/>
</segment>
</net>
<net name="CLTCH-H_OUT" class="3">
<segment>
<pinref part="I/O" gate="G$1" pin="P$14"/>
<wire x1="27.94" y1="20.32" x2="22.86" y2="20.32" width="0.1524" layer="91"/>
<label x="5.08" y="20.32" size="1.778" layer="95"/>
</segment>
</net>
<net name="BL_MO" class="1">
<segment>
<pinref part="I/O" gate="G$1" pin="P$5"/>
<wire x1="30.48" y1="27.94" x2="22.86" y2="27.94" width="0.1524" layer="91"/>
<label x="5.08" y="27.94" size="1.778" layer="95"/>
</segment>
</net>
<net name="FAN_MO" class="1">
<segment>
<pinref part="I/O" gate="G$1" pin="P$1"/>
<wire x1="30.48" y1="50.8" x2="22.86" y2="50.8" width="0.1524" layer="91"/>
<label x="5.08" y="50.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="DNS_MO" class="1">
<segment>
<pinref part="I/O" gate="G$1" pin="P$4"/>
<wire x1="30.48" y1="35.56" x2="22.86" y2="35.56" width="0.1524" layer="91"/>
<label x="5.08" y="35.56" size="1.778" layer="95"/>
</segment>
</net>
<net name="CLTCH-H_MO" class="1">
<segment>
<pinref part="I/O" gate="G$1" pin="P$6"/>
<wire x1="30.48" y1="22.86" x2="22.86" y2="22.86" width="0.1524" layer="91"/>
<label x="5.08" y="22.86" size="1.778" layer="95"/>
</segment>
</net>
<net name="UPS_OUT" class="3">
<segment>
<pinref part="I/O" gate="G$1" pin="P$11"/>
<wire x1="27.94" y1="38.1" x2="22.86" y2="38.1" width="0.1524" layer="91"/>
<label x="5.08" y="38.1" size="1.778" layer="95"/>
</segment>
</net>
<net name="UPS_MO" class="1">
<segment>
<pinref part="I/O" gate="G$1" pin="P$3"/>
<wire x1="30.48" y1="40.64" x2="22.86" y2="40.64" width="0.1524" layer="91"/>
<label x="5.08" y="40.64" size="1.778" layer="95"/>
</segment>
</net>
<net name="CLTCH_OUT" class="3">
<segment>
<pinref part="I/O" gate="G$1" pin="P$15"/>
<wire x1="27.94" y1="15.24" x2="22.86" y2="15.24" width="0.1524" layer="91"/>
<label x="5.08" y="15.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="CLTCH_MO" class="1">
<segment>
<pinref part="I/O" gate="G$1" pin="P$7"/>
<wire x1="30.48" y1="17.78" x2="22.86" y2="17.78" width="0.1524" layer="91"/>
<label x="5.08" y="17.78" size="1.778" layer="95"/>
</segment>
</net>
<net name="FP&amp;IGN_OUT-F" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="P1"/>
<wire x1="96.52" y1="-17.78" x2="99.06" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="99.06" y1="-17.78" x2="99.06" y2="-10.16" width="0.1524" layer="91"/>
<pinref part="U$2" gate="G$1" pin="P2"/>
<wire x1="99.06" y1="-10.16" x2="96.52" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="99.06" y1="-10.16" x2="99.06" y2="-2.54" width="0.1524" layer="91"/>
<junction x="99.06" y="-10.16"/>
<pinref part="U$2" gate="G$1" pin="P3"/>
<wire x1="99.06" y1="-2.54" x2="96.52" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="99.06" y1="-2.54" x2="99.06" y2="5.08" width="0.1524" layer="91"/>
<junction x="99.06" y="-2.54"/>
<pinref part="U$2" gate="G$1" pin="P4"/>
<wire x1="99.06" y1="5.08" x2="96.52" y2="5.08" width="0.1524" layer="91"/>
<label x="101.6" y="-10.16" size="1.778" layer="95" rot="R90"/>
</segment>
<segment>
<pinref part="I/O" gate="G$1" pin="P$10"/>
<wire x1="27.94" y1="43.18" x2="22.86" y2="43.18" width="0.1524" layer="91"/>
<label x="5.08" y="43.18" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="I/O" gate="G$1" pin="P$9"/>
<wire x1="27.94" y1="48.26" x2="22.86" y2="48.26" width="0.1524" layer="91"/>
<label x="5.08" y="48.26" size="1.778" layer="95"/>
</segment>
</net>
<net name="E_STOP" class="1">
<segment>
<pinref part="I/O" gate="G$1" pin="P$2"/>
<wire x1="30.48" y1="45.72" x2="22.86" y2="45.72" width="0.1524" layer="91"/>
<label x="5.08" y="45.72" size="1.778" layer="95"/>
</segment>
</net>
</nets>
</sheet>
<sheet>
<plain>
<text x="33.02" y="10.16" size="1.778" layer="91" rot="MR180">g</text>
<text x="40.64" y="0" size="1.778" layer="91" rot="MR180">d</text>
<text x="40.64" y="10.16" size="1.778" layer="91" rot="MR180">s</text>
</plain>
<instances>
<instance part="FP&amp;IGN_D1" gate="G$1" x="-20.32" y="15.24" rot="R90"/>
<instance part="FP&amp;IGN_R" gate="G$1" x="10.16" y="25.4" rot="R90"/>
<instance part="FP&amp;IGN_D2" gate="G$1" x="-20.32" y="-5.08" rot="R90"/>
<instance part="FP&amp;IGN" gate="G$1" x="38.1" y="5.08" rot="MR180"/>
</instances>
<busses>
</busses>
<nets>
<net name="E_STOP" class="1">
<segment>
<pinref part="FP&amp;IGN_D2" gate="G$1" pin="P$2"/>
<pinref part="FP&amp;IGN_D1" gate="G$1" pin="P$1"/>
<wire x1="-20.32" y1="5.08" x2="-20.32" y2="7.62" width="0.1524" layer="91"/>
<wire x1="-20.32" y1="7.62" x2="-20.32" y2="10.16" width="0.1524" layer="91"/>
<junction x="-20.32" y="7.62"/>
<pinref part="FP&amp;IGN_R" gate="G$1" pin="P$2"/>
<wire x1="10.16" y1="7.62" x2="-20.32" y2="7.62" width="0.1524" layer="91"/>
<wire x1="10.16" y1="15.24" x2="10.16" y2="7.62" width="0.1524" layer="91"/>
<pinref part="FP&amp;IGN" gate="G$1" pin="G"/>
<wire x1="33.02" y1="7.62" x2="10.16" y2="7.62" width="0.1524" layer="91"/>
<junction x="10.16" y="7.62"/>
<wire x1="-20.32" y1="7.62" x2="-53.34" y2="7.62" width="0.1524" layer="91"/>
<label x="-53.34" y="8.128" size="1.778" layer="95"/>
</segment>
</net>
<net name="GND" class="2">
<segment>
<pinref part="FP&amp;IGN_D2" gate="G$1" pin="P$1"/>
<wire x1="-20.32" y1="-10.16" x2="-20.32" y2="-45.72" width="0.1524" layer="91"/>
<label x="-17.78" y="-43.18" size="1.778" layer="95"/>
</segment>
</net>
<net name="+12VS" class="2">
<segment>
<pinref part="FP&amp;IGN_R" gate="G$1" pin="P$1"/>
<wire x1="10.16" y1="35.56" x2="10.16" y2="43.18" width="0.1524" layer="91"/>
<wire x1="10.16" y1="43.18" x2="43.18" y2="43.18" width="0.1524" layer="91"/>
<pinref part="FP&amp;IGN_D1" gate="G$1" pin="P$2"/>
<wire x1="-20.32" y1="25.4" x2="-20.32" y2="43.18" width="0.1524" layer="91"/>
<wire x1="-20.32" y1="43.18" x2="10.16" y2="43.18" width="0.1524" layer="91"/>
<junction x="10.16" y="43.18"/>
<label x="38.1" y="45.72" size="1.778" layer="95"/>
<pinref part="FP&amp;IGN" gate="G$1" pin="S"/>
<wire x1="43.18" y1="7.62" x2="43.18" y2="43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="FP&amp;IGN_OUT" class="3">
<segment>
<pinref part="FP&amp;IGN" gate="G$1" pin="D"/>
<wire x1="43.18" y1="2.54" x2="43.18" y2="-45.72" width="0.1524" layer="91"/>
<label x="48.26" y="-45.72" size="1.778" layer="95"/>
</segment>
</net>
</nets>
</sheet>
<sheet>
<plain>
</plain>
<instances>
<instance part="UPS_R" gate="G$1" x="0" y="17.78" rot="R90"/>
<instance part="UPS_D1" gate="G$1" x="-17.78" y="7.62" rot="R90"/>
<instance part="UPS_D2" gate="G$1" x="-17.78" y="-12.7" rot="R90"/>
<instance part="UPS" gate="G$1" x="17.78" y="-2.54" rot="MR180"/>
<instance part="DNS_R" gate="G$1" x="86.36" y="17.78" rot="R90"/>
<instance part="DNS_D1" gate="G$1" x="68.58" y="7.62" rot="R90"/>
<instance part="DNS_D2" gate="G$1" x="68.58" y="-12.7" rot="R90"/>
<instance part="DNS" gate="G$1" x="104.14" y="-2.54" rot="MR180"/>
<instance part="CLTCH_R" gate="G$1" x="175.26" y="17.78" rot="R90"/>
<instance part="CLTCH_D1" gate="G$1" x="157.48" y="7.62" rot="R90"/>
<instance part="CLTCH_D2" gate="G$1" x="157.48" y="-12.7" rot="R90"/>
<instance part="CLTCH" gate="G$1" x="193.04" y="-2.54" rot="MR180"/>
<instance part="CLTCH-H_R" gate="G$1" x="-2.54" y="-55.88" rot="R90"/>
<instance part="CLTCH-H_D1" gate="G$1" x="-20.32" y="-66.04" rot="R90"/>
<instance part="CLTCH-H_D2" gate="G$1" x="-20.32" y="-86.36" rot="R90"/>
<instance part="CLTCH-H" gate="G$1" x="15.24" y="-76.2" rot="MR180"/>
<instance part="BL_R" gate="G$1" x="86.36" y="-55.88" rot="R90"/>
<instance part="BL_D1" gate="G$1" x="68.58" y="-66.04" rot="R90"/>
<instance part="BL_D2" gate="G$1" x="68.58" y="-86.36" rot="R90"/>
<instance part="BL" gate="G$1" x="104.14" y="-76.2" rot="MR180"/>
<instance part="FAN_R" gate="G$1" x="175.26" y="-53.34" rot="R90"/>
<instance part="FAN_D1" gate="G$1" x="157.48" y="-63.5" rot="R90"/>
<instance part="FAN_D2" gate="G$1" x="157.48" y="-83.82" rot="R90"/>
<instance part="FAN" gate="G$1" x="193.04" y="-73.66" rot="MR180"/>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="2">
<segment>
<pinref part="UPS_D2" gate="G$1" pin="P$1"/>
<wire x1="-17.78" y1="-17.78" x2="-17.78" y2="-25.4" width="0.1524" layer="91"/>
<label x="-17.78" y="-25.4" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="DNS_D2" gate="G$1" pin="P$1"/>
<wire x1="68.58" y1="-17.78" x2="68.58" y2="-25.4" width="0.1524" layer="91"/>
<label x="68.58" y="-25.4" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="CLTCH_D2" gate="G$1" pin="P$1"/>
<wire x1="157.48" y1="-17.78" x2="157.48" y2="-25.4" width="0.1524" layer="91"/>
<label x="157.48" y="-25.4" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="CLTCH-H_D2" gate="G$1" pin="P$1"/>
<wire x1="-20.32" y1="-91.44" x2="-20.32" y2="-99.06" width="0.1524" layer="91"/>
<label x="-20.32" y="-99.06" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="BL_D2" gate="G$1" pin="P$1"/>
<wire x1="68.58" y1="-91.44" x2="68.58" y2="-99.06" width="0.1524" layer="91"/>
<label x="68.58" y="-99.06" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="FAN_D2" gate="G$1" pin="P$1"/>
<wire x1="157.48" y1="-88.9" x2="157.48" y2="-96.52" width="0.1524" layer="91"/>
<label x="157.48" y="-96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="+12VS" class="2">
<segment>
<pinref part="UPS_D1" gate="G$1" pin="P$2"/>
<wire x1="-17.78" y1="17.78" x2="-17.78" y2="30.48" width="0.1524" layer="91"/>
<wire x1="-17.78" y1="30.48" x2="0" y2="30.48" width="0.1524" layer="91"/>
<wire x1="0" y1="30.48" x2="22.86" y2="30.48" width="0.1524" layer="91"/>
<pinref part="UPS_R" gate="G$1" pin="P$1"/>
<wire x1="0" y1="27.94" x2="0" y2="30.48" width="0.1524" layer="91"/>
<junction x="0" y="30.48"/>
<label x="10.16" y="30.48" size="1.778" layer="95"/>
<wire x1="22.86" y1="30.48" x2="22.86" y2="0" width="0.1524" layer="91"/>
<pinref part="UPS" gate="G$1" pin="S"/>
</segment>
<segment>
<pinref part="DNS_D1" gate="G$1" pin="P$2"/>
<wire x1="68.58" y1="17.78" x2="68.58" y2="30.48" width="0.1524" layer="91"/>
<wire x1="68.58" y1="30.48" x2="86.36" y2="30.48" width="0.1524" layer="91"/>
<wire x1="86.36" y1="30.48" x2="109.22" y2="30.48" width="0.1524" layer="91"/>
<pinref part="DNS_R" gate="G$1" pin="P$1"/>
<wire x1="86.36" y1="27.94" x2="86.36" y2="30.48" width="0.1524" layer="91"/>
<junction x="86.36" y="30.48"/>
<label x="96.52" y="30.48" size="1.778" layer="95"/>
<wire x1="109.22" y1="30.48" x2="109.22" y2="0" width="0.1524" layer="91"/>
<pinref part="DNS" gate="G$1" pin="S"/>
</segment>
<segment>
<pinref part="CLTCH_D1" gate="G$1" pin="P$2"/>
<wire x1="157.48" y1="17.78" x2="157.48" y2="30.48" width="0.1524" layer="91"/>
<wire x1="157.48" y1="30.48" x2="175.26" y2="30.48" width="0.1524" layer="91"/>
<wire x1="175.26" y1="30.48" x2="198.12" y2="30.48" width="0.1524" layer="91"/>
<pinref part="CLTCH_R" gate="G$1" pin="P$1"/>
<wire x1="175.26" y1="27.94" x2="175.26" y2="30.48" width="0.1524" layer="91"/>
<junction x="175.26" y="30.48"/>
<label x="185.42" y="30.48" size="1.778" layer="95"/>
<wire x1="198.12" y1="30.48" x2="198.12" y2="0" width="0.1524" layer="91"/>
<pinref part="CLTCH" gate="G$1" pin="S"/>
</segment>
<segment>
<pinref part="CLTCH-H_D1" gate="G$1" pin="P$2"/>
<wire x1="-20.32" y1="-55.88" x2="-20.32" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="-20.32" y1="-43.18" x2="-2.54" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="-2.54" y1="-43.18" x2="20.32" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="CLTCH-H_R" gate="G$1" pin="P$1"/>
<wire x1="-2.54" y1="-45.72" x2="-2.54" y2="-43.18" width="0.1524" layer="91"/>
<junction x="-2.54" y="-43.18"/>
<label x="7.62" y="-43.18" size="1.778" layer="95"/>
<wire x1="20.32" y1="-43.18" x2="20.32" y2="-73.66" width="0.1524" layer="91"/>
<pinref part="CLTCH-H" gate="G$1" pin="S"/>
</segment>
<segment>
<pinref part="BL_D1" gate="G$1" pin="P$2"/>
<wire x1="68.58" y1="-55.88" x2="68.58" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="68.58" y1="-43.18" x2="86.36" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="86.36" y1="-43.18" x2="109.22" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="BL_R" gate="G$1" pin="P$1"/>
<wire x1="86.36" y1="-45.72" x2="86.36" y2="-43.18" width="0.1524" layer="91"/>
<junction x="86.36" y="-43.18"/>
<label x="96.52" y="-43.18" size="1.778" layer="95"/>
<wire x1="109.22" y1="-43.18" x2="109.22" y2="-73.66" width="0.1524" layer="91"/>
<pinref part="BL" gate="G$1" pin="S"/>
</segment>
<segment>
<pinref part="FAN_D1" gate="G$1" pin="P$2"/>
<wire x1="157.48" y1="-53.34" x2="157.48" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="157.48" y1="-40.64" x2="175.26" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="175.26" y1="-40.64" x2="198.12" y2="-40.64" width="0.1524" layer="91"/>
<pinref part="FAN_R" gate="G$1" pin="P$1"/>
<wire x1="175.26" y1="-43.18" x2="175.26" y2="-40.64" width="0.1524" layer="91"/>
<junction x="175.26" y="-40.64"/>
<label x="185.42" y="-40.64" size="1.778" layer="95"/>
<wire x1="198.12" y1="-40.64" x2="198.12" y2="-71.12" width="0.1524" layer="91"/>
<pinref part="FAN" gate="G$1" pin="S"/>
</segment>
</net>
<net name="UPS_MO" class="1">
<segment>
<pinref part="UPS_D1" gate="G$1" pin="P$1"/>
<pinref part="UPS_D2" gate="G$1" pin="P$2"/>
<wire x1="-17.78" y1="2.54" x2="-17.78" y2="0" width="0.1524" layer="91"/>
<wire x1="-17.78" y1="0" x2="-17.78" y2="-2.54" width="0.1524" layer="91"/>
<junction x="-17.78" y="0"/>
<pinref part="UPS_R" gate="G$1" pin="P$2"/>
<wire x1="0" y1="0" x2="-17.78" y2="0" width="0.1524" layer="91"/>
<wire x1="0" y1="7.62" x2="0" y2="0" width="0.1524" layer="91"/>
<wire x1="-17.78" y1="0" x2="-40.64" y2="0" width="0.1524" layer="91"/>
<label x="-40.64" y="0" size="1.778" layer="95"/>
<pinref part="UPS" gate="G$1" pin="G"/>
<wire x1="0" y1="0" x2="12.7" y2="0" width="0.1524" layer="91"/>
<junction x="0" y="0"/>
</segment>
</net>
<net name="UPS_OUT" class="3">
<segment>
<pinref part="UPS" gate="G$1" pin="D"/>
<wire x1="22.86" y1="-5.08" x2="22.86" y2="-25.4" width="0.1524" layer="91"/>
<label x="25.4" y="-25.4" size="1.778" layer="95"/>
</segment>
</net>
<net name="DNS_OUT" class="3">
<segment>
<pinref part="DNS" gate="G$1" pin="D"/>
<wire x1="109.22" y1="-5.08" x2="109.22" y2="-25.4" width="0.1524" layer="91"/>
<label x="111.76" y="-25.4" size="1.778" layer="95"/>
</segment>
</net>
<net name="CLTCH_OUT" class="3">
<segment>
<pinref part="CLTCH" gate="G$1" pin="D"/>
<wire x1="198.12" y1="-5.08" x2="198.12" y2="-25.4" width="0.1524" layer="91"/>
<label x="203.2" y="-25.4" size="1.778" layer="95"/>
</segment>
</net>
<net name="CLTCH-H_OUT" class="3">
<segment>
<pinref part="CLTCH-H" gate="G$1" pin="D"/>
<wire x1="20.32" y1="-78.74" x2="20.32" y2="-99.06" width="0.1524" layer="91"/>
<label x="25.4" y="-99.06" size="1.778" layer="95"/>
</segment>
</net>
<net name="BL_OUT" class="3">
<segment>
<pinref part="BL" gate="G$1" pin="D"/>
<wire x1="109.22" y1="-78.74" x2="109.22" y2="-99.06" width="0.1524" layer="91"/>
<label x="114.3" y="-99.06" size="1.778" layer="95"/>
</segment>
</net>
<net name="FAN_OUT" class="2">
<segment>
<pinref part="FAN" gate="G$1" pin="D"/>
<wire x1="198.12" y1="-76.2" x2="198.12" y2="-96.52" width="0.1524" layer="91"/>
<label x="203.2" y="-96.52" size="1.778" layer="95"/>
</segment>
</net>
<net name="DNS_MO" class="1">
<segment>
<pinref part="DNS_D1" gate="G$1" pin="P$1"/>
<pinref part="DNS_D2" gate="G$1" pin="P$2"/>
<wire x1="68.58" y1="2.54" x2="68.58" y2="0" width="0.1524" layer="91"/>
<wire x1="68.58" y1="0" x2="68.58" y2="-2.54" width="0.1524" layer="91"/>
<junction x="68.58" y="0"/>
<pinref part="DNS_R" gate="G$1" pin="P$2"/>
<wire x1="86.36" y1="0" x2="68.58" y2="0" width="0.1524" layer="91"/>
<wire x1="86.36" y1="7.62" x2="86.36" y2="0" width="0.1524" layer="91"/>
<wire x1="68.58" y1="0" x2="45.72" y2="0" width="0.1524" layer="91"/>
<label x="45.72" y="0" size="1.778" layer="95"/>
<pinref part="DNS" gate="G$1" pin="G"/>
<wire x1="86.36" y1="0" x2="99.06" y2="0" width="0.1524" layer="91"/>
<junction x="86.36" y="0"/>
</segment>
</net>
<net name="CLTCH_MO" class="1">
<segment>
<pinref part="CLTCH_D1" gate="G$1" pin="P$1"/>
<pinref part="CLTCH_D2" gate="G$1" pin="P$2"/>
<wire x1="157.48" y1="2.54" x2="157.48" y2="0" width="0.1524" layer="91"/>
<wire x1="157.48" y1="0" x2="157.48" y2="-2.54" width="0.1524" layer="91"/>
<junction x="157.48" y="0"/>
<pinref part="CLTCH_R" gate="G$1" pin="P$2"/>
<wire x1="175.26" y1="0" x2="157.48" y2="0" width="0.1524" layer="91"/>
<wire x1="175.26" y1="7.62" x2="175.26" y2="0" width="0.1524" layer="91"/>
<wire x1="157.48" y1="0" x2="134.62" y2="0" width="0.1524" layer="91"/>
<label x="134.62" y="0" size="1.778" layer="95"/>
<pinref part="CLTCH" gate="G$1" pin="G"/>
<wire x1="175.26" y1="0" x2="187.96" y2="0" width="0.1524" layer="91"/>
<junction x="175.26" y="0"/>
</segment>
</net>
<net name="CLTCH-H_MO" class="1">
<segment>
<pinref part="CLTCH-H_D1" gate="G$1" pin="P$1"/>
<pinref part="CLTCH-H_D2" gate="G$1" pin="P$2"/>
<wire x1="-20.32" y1="-71.12" x2="-20.32" y2="-73.66" width="0.1524" layer="91"/>
<wire x1="-20.32" y1="-73.66" x2="-20.32" y2="-76.2" width="0.1524" layer="91"/>
<junction x="-20.32" y="-73.66"/>
<pinref part="CLTCH-H_R" gate="G$1" pin="P$2"/>
<wire x1="-2.54" y1="-73.66" x2="-20.32" y2="-73.66" width="0.1524" layer="91"/>
<wire x1="-2.54" y1="-66.04" x2="-2.54" y2="-73.66" width="0.1524" layer="91"/>
<wire x1="-20.32" y1="-73.66" x2="-43.18" y2="-73.66" width="0.1524" layer="91"/>
<label x="-43.18" y="-73.66" size="1.778" layer="95"/>
<pinref part="CLTCH-H" gate="G$1" pin="G"/>
<wire x1="-2.54" y1="-73.66" x2="10.16" y2="-73.66" width="0.1524" layer="91"/>
<junction x="-2.54" y="-73.66"/>
</segment>
</net>
<net name="BL_MO" class="1">
<segment>
<pinref part="BL_D1" gate="G$1" pin="P$1"/>
<pinref part="BL_D2" gate="G$1" pin="P$2"/>
<wire x1="68.58" y1="-71.12" x2="68.58" y2="-73.66" width="0.1524" layer="91"/>
<wire x1="68.58" y1="-73.66" x2="68.58" y2="-76.2" width="0.1524" layer="91"/>
<junction x="68.58" y="-73.66"/>
<pinref part="BL_R" gate="G$1" pin="P$2"/>
<wire x1="86.36" y1="-73.66" x2="68.58" y2="-73.66" width="0.1524" layer="91"/>
<wire x1="86.36" y1="-66.04" x2="86.36" y2="-73.66" width="0.1524" layer="91"/>
<wire x1="68.58" y1="-73.66" x2="45.72" y2="-73.66" width="0.1524" layer="91"/>
<label x="45.72" y="-73.66" size="1.778" layer="95"/>
<pinref part="BL" gate="G$1" pin="G"/>
<wire x1="86.36" y1="-73.66" x2="99.06" y2="-73.66" width="0.1524" layer="91"/>
<junction x="86.36" y="-73.66"/>
</segment>
</net>
<net name="FAN_MO" class="1">
<segment>
<pinref part="FAN_D1" gate="G$1" pin="P$1"/>
<pinref part="FAN_D2" gate="G$1" pin="P$2"/>
<wire x1="157.48" y1="-68.58" x2="157.48" y2="-71.12" width="0.1524" layer="91"/>
<wire x1="157.48" y1="-71.12" x2="157.48" y2="-73.66" width="0.1524" layer="91"/>
<junction x="157.48" y="-71.12"/>
<pinref part="FAN_R" gate="G$1" pin="P$2"/>
<wire x1="175.26" y1="-71.12" x2="157.48" y2="-71.12" width="0.1524" layer="91"/>
<wire x1="175.26" y1="-63.5" x2="175.26" y2="-71.12" width="0.1524" layer="91"/>
<wire x1="157.48" y1="-71.12" x2="134.62" y2="-71.12" width="0.1524" layer="91"/>
<label x="134.62" y="-71.12" size="1.778" layer="95"/>
<pinref part="FAN" gate="G$1" pin="G"/>
<wire x1="175.26" y1="-71.12" x2="187.96" y2="-71.12" width="0.1524" layer="91"/>
<junction x="175.26" y="-71.12"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
